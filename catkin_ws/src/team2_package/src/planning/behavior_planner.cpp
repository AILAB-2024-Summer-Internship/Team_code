#include "behavior_planner.hpp"

CollisionCheck::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0) {}
CollisionCheck::object::object(float min_y, float min_x, float max_y, float max_x) : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x) {}

CollisionCheck::CollisionCheck() {
    object_sub = nh.subscribe("/bounding_box", 10, &CollisionCheck::object_cb, this);
    road_option_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &CollisionCheck::road_option_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &CollisionCheck::speed_cb, this);
    stop_pub = nh.advertise<std_msgs::Bool>("/carla/hero/global_stop", 10);
    objects.reserve(500);
}

void CollisionCheck::object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg) {
    objects.clear();
    size_t size = msg->boxes.size();
    for (int i = 0; i < size; i++) {
        float min_y = msg->boxes[i].center.y - (msg->boxes[i].size_y / 2);
        float min_x = msg->boxes[i].center.x - (msg->boxes[i].size_x / 2);
        float max_y = msg->boxes[i].center.y + (msg->boxes[i].size_y / 2);
        float max_x = msg->boxes[i].center.x + (msg->boxes[i].size_x / 2);
        objects.emplace_back(object(min_y, min_x, max_y, max_x));
    }
}
void CollisionCheck::road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg) {
    next_rop = msg->road_options[0];
    next2_rop = msg->road_options[1];
}
void CollisionCheck::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
}
void CollisionCheck::localplan_cb(const std_msgs::Bool::ConstPtr& msg) {
    local_plan = msg->data; // if true -> localplanning
}

void CollisionCheck::object_detect(const int& next_rop, const int& speed, const std::vector<object>& objects) {
    size_t size = objects.size();
    if (next_rop == 3 || next_rop == 4) {
        for(int i = 0; i < size; i++) {
            float leftob = objects[i].max_y;
            float rightob = objects[i].min_y;
            float frontob = objects[i].min_x;
            float vehicle_max_x = 1.5 * speed + 2.5;

            if(((2.5 <= frontob && frontob <= vehicle_max_x) && (-1 <= rightob && rightob <= 1)) ||
                ((2.5 <= frontob && frontob <= vehicle_max_x) && (-1 <= leftob && leftob <= 1))) {
                object_detected = true;
                // if (objects[i].min_y >= 0) {
                //     ros::Duration(3.0).sleep();
                //     if(objects[i].min_y >= 0) {
                //         local_plan = true;
                //     }
                // }
            } else {
                object_detected = false;
            }
        }
    // else if (nextrop == 1 || next_rop == 2) {
    // } else if (nextrop == 5 || next_rop == 6) {
    //     local_plan = true;
    } else {
        object_detected = false;
    }
    std::cout << "detecting" << std::endl;
}

void CollisionCheck::junction_intersection(const int& next_rop, const int& next2_rop)  {
    if (next_rop == 4 && next2_rop != 4) {
        jcic_stop = true;
        ros::Duration(1.0).sleep();
        if(trafficlight_none || green_light) {
            jcic_stop = false;
        }
    }
    std::cout << "junction" << std::endl;
}

void CollisionCheck::stop_check() {
    if (local_plan == false) {
        object_detect(next_rop, speed, objects);
        junction_intersection(next_rop, next2_rop);
        if (object_detected || jcic_stop) {
            g_stop = true;
        } else {
            g_stop = false;
        }
    } else {
        object_detected = false;
        jcic_stop = false;
        trafficlight_none = true;
        green_light = false;
        g_stop = false;
    }
    std::cout << "stop checking" << std::endl;
}

void CollisionCheck::global_stop_publisher() {
    stop_check();
    std_msgs::Bool stopmsg;
    stopmsg.data = g_stop;
    stop_pub.publish(stopmsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_planner");
    CollisionCheck collisioncheck;
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        collisioncheck.global_stop_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}