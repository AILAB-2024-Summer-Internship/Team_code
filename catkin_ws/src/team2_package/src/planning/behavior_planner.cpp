#include "behavior_planner.hpp"

CollisionCheck::object::object() : min_x(0.0), min_y(0.0), max_x(0.0), max_y(0.0) {}
CollisionCheck::object::object(float min_x, float min_y, float max_x, float max_y) : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {}

CollisionCheck::CollisionCheck() {
    object_sub = nh.subscribe("bounding_box", 10, &CollisionCheck::object_sub, this);
    road_option_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &CollisionCheck::road_option_sub, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &CollisionCheck::speed_sub, this);
    stop_pub = nh.advertise<std_msgs::Bool>("/carla/hero/global_stop", 10);
    objects.reserve(100);
}

void CollisionCheck::object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg) {
    size_t size = msg->.size();
    for (int i = 0 i < size; i++) {
        float min_x = msg->boxes[i].center.x - msg->boxes[i].size_x / 2;
        float min_y = msg->boxes[i].center.y - msg->boxes[i].size_y / 2;
        float max_x = msg->boxes[i].center.x + msg->boxes[i].size_x / 2;
        float max_y = msg->boxes[i].center.y + msg->boxes[i].size_y / 2;
        objects.emplace_back(object(min_x, min_y, max_x, max_y));
    }
    std::cout<< min_x[0] << std::endl;
}
void CollisionCheck::road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg) {
    next_rop = msg->road_options[0];
    next2_rop = msg->road_options[1];
}
void CollisionCheck::speed_cb(const carla_msgs::Speed::ConstPtr& msg) {
    speed = msg->speed;
}
void CollisionCheck::localplan_cb(const std_msgs::Bool::ConstPtr& msg) {
    local_plan = msg->data; // if true -> localplanning
}

void CollisionCheck::object_detect(const int& next_rop, const std::vector<object>& objects) {
    size_t size = objects.size();
    if (next_rop == 3 || next_rop == 4) {
        for(int i = 0; i < size; i++) {
            if((2.5 <= objects[i].min_x <= 1.5 * speed + 2.5) && (-1 <= objects[i].min_y <= 1)) {
                object_detected = true;
                if (objects[i].min_y >= 0) {
                    ros::Duration(3.0).sleep();
                    if(objects[i].min_y >= 0) {
                        local_plan = true;
                    }
                }
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
}

void CollisionCheck::junction_intersection(const int& next_rop, const int& next2_rop)  {
    if (next_rop == 4 && next2_rop != 4) {
        jcic_stop = true;
        ros::Duration(1.0).sleep();
        if(trafficlight_none || green_light) {
            jcic_stop = false;
        }
    }
}

void CollisionCheck::stop() {
    if (local_plan == false) {
        object_detect(next_rop, objects);
        junction_intersection(next_rop, next2_rop);
        if (object_detected || jcic_stop) {
            global_stop = true;
        } else {
            global_stop = false;
        }
    } else {
        object_detected = false;
        jcic_stop = false;
        trafficlight_none = true;
        green_light = false;
        global_stop = false;
    }
}

void CollisionCheck::global_stop_publisher() {
    std_msgs::Bool stopmsg;
    stopmsg = global_stop;
    stop_pub.publish(stopmsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_planner");
    CollisionCheck collisioncheck;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        collisioncheck.stop();
        collisioncheck.global_stop_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
