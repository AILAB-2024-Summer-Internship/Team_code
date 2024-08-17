#include "behavior_planner.hpp"

BehaviorPlanner::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0), v_y(0.0), v_x(0.0) {}
BehaviorPlanner::object::object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x) : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x), v_y(v_y), v_x(v_x) {}

BehaviorPlanner::waypoint::waypoint() : x(0.0), y(0.0), speed(0.0) {}
BehaviorPlanner::waypoint::waypoint(float x, float y, float speed) : x(x), y(y), speed(speed) {}

BehaviorPlanner::BehaviorPlanner() {
    object_sub = nh.subscribe("/pub_planner_bbox", 10, &BehaviorPlanner::object_cb, this);
    // waypoint_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &BehaviorPlanner::waypoint_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &BehaviorPlanner::speed_cb, this);
    // pose_sub = nh.subscribe("/carla/hero/localization", 10, &CollisionCheck::pose_cb, this);
    
    AEB_pub = nh.advertise<std_msgs::Bool>("/carla/hero/AEB", 10);

    objects.reserve(500);
    pose.reserve(3);
}

void BehaviorPlanner::object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg) {
    objects.clear();
    size_t size = msg->boxes.size();
    for (int i = 0; i < size; i++) {
        float min_y = msg->boxes[i].center.y - (msg->boxes[i].size_y / 2);
        float min_x = msg->boxes[i].center.x - (msg->boxes[i].size_x / 2);
        float max_y = msg->boxes[i].center.y + (msg->boxes[i].size_y / 2);
        float max_x = msg->boxes[i].center.x + (msg->boxes[i].size_x / 2);
        float v_y = 0.0;
        float v_x = 0.0;
        objects.push_back(object(min_y, min_x, max_y, max_x, v_y, v_x));
    }
    if (size != 0) {
    // object_prediction(objects);
        collision_check(objects, speed);
    }
}

// void BehaviorPlanner::waypoint_cb(const team2_package::globalwaypoints::ConstPtr& msg) {
//     size_t size = road_options.size();
//     for (int i = 0; i < size; i++) {
//         waypoints[i].x = msg->x[i];
//         waypoints[i].y = msg->y[i];
//     }
//     road_option = road_options[0];
// }

// void BehaviorPlanner::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
//     pose[0] = msg->x;
//     pose[1] = msg->y;
//     pose[2] = msg->yaw;
// }

void BehaviorPlanner::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
}

// void BehaviorPlanner::object_prediction(const std::vector<object>& objects) {
//     for(int i = 0; i < objects.size(); i++) {
//         std::vector<object> predict_3s;
//         predict_3s.reserve(10);
//         for(int j = 0; j < 10; j++) {
//             float min_path_y = objects[i].min_y + 0.3 * j * objects[i].v_y;
//             float max_path_y = objects[i].max_y + 0.3 * j * objects[i].v_y;
//             float min_path_x = objects[i].min_x + 0.3 * j * objects[i].v_x;
//             float max_path_x = objects[i].max_x + 0.3 * j * objects[i].v_x;
//             predict_3s.push_back(object(min_path_y, max_path_y, min_path_x, min_path_y, 0, 0));
//         }
//         objects_predict_3s.emplace_back(predict_3s);
//     }
// }

// void BehaviorPlanner::collision_check(const std::vector<object>& objects_predict_3s) {
//     float my_x = 
// }

void BehaviorPlanner::collision_check(const std::vector<object>& objects, const float& speed) { // min_max add
    size_t size = objects.size();
    bool AEB_loop = false;
    for (int i = 0; i < size; i++) {
        float min_x = objects[i].min_x;
        float min_y = objects[i].min_y;
        float max_y = objects[i].max_y;
        if (speed > 3.0) {
            if ((2.50 < min_x && min_x < 2.50 + 1.5 * speed) &&
            (((max_y >= -1) && (min_y < -1)) || ((min_y < 1) && (max_y > 1)) || (-1 < min_y && max_y < 1))) {
                AEB_loop = true;
                break;
            } else {
                AEB_loop = false;
            }
        } else {
            if ((2.50 < min_x && min_x < 7.0) &&
            (((max_y >= -1) && (min_y < -1)) || ((min_y < 1) && (max_y > 1)) || (-1 < min_y && max_y < 1))) {
                AEB_loop = true;
                break;
            } else {
                AEB_loop = false;
            }
        }
    }
    if (AEB_loop) {
        AEB = true;
    } else if (AEB && !AEB_loop) {
        AEB = false;
    }
}

void BehaviorPlanner::publisher() {
    std_msgs::Bool msg;
    msg.data = AEB;
    AEB_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_planner");
    BehaviorPlanner behaviorplanner;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        behaviorplanner.publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
