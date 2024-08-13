#include "behavior_planner.hpp"

BehaviorPlanner::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0), v_y(0.0), v_x(0,0) {}
BehaviorPlanner::object::object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x) : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x), v_y(v_y), v_x(v_x) {}

BehaviorPlanner::BehaviorPlanner() {
    object_sub = nh.subscribe("/bounding_box", 10, &CollisionCheck::object_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &CollisionCheck::speed_cb, this);
    yaw_sub = nh.subscribe("/carla/hero/localization", 10, &CollisionCheck::yaw_cb, this);
    steering_angle_sub = nh.subscribe("")
    road_option_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &CollisionCheck::road_option_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &CollisionCheck::speed_cb, this);
    // local_fin_sub = nh.subscribe("/carla/hero/local_fin", 10, &CollisionCheck::local_fin_cb, this);
    // traffic_sign_sub = nh.subscrbie("/carla/traffic_sign"), 10, &CollisionCheck::traffic_sign_cb, this);
    stop_pub = nh.advertise<std_msgs::Bool>("/carla/hero/stop", 10);
    // local_req_pub = nh.advertise<std_msgs::Bool>("/localplan_request", 10);
    // acc_speed_pub = nh.advertise<carla_msgs::CarlaSpeedometer>("/localplan_request", 10);
    objects.reserve(500);
    maybe.reserve(50);
}

void CollisionCheck::object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg) {
    objects.clear();
    size_t size = msg->boxes.size();
    for (int i = 0; i < size; i++) {
        float min_y = msg->boxes[i].center.y - (msg->boxes[i].size_y / 2);
        float min_x = msg->boxes[i].center.x - (msg->boxes[i].size_x / 2);
        float max_y = msg->boxes[i].center.y + (msg->boxes[i].size_y / 2);
        float max_x = msg->boxes[i].center.x + (msg->boxes[i].size_x / 2);
        float v_y = 0.0
        float v_x = 0.0
        objects.push_back(object(min_y, min_x, max_y, max_x, v_y, v_x));
    }
    object_path_prediction(objects);
}
void CollisionCheck::road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg) { //necessary?
    next_rop = msg->road_options[0];
    next2_rop = msg->road_options[1];
}
void CollisionCheck::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
}
// void CollisionCheck::local_fin_cb(const std_msgs::Bool::ConstPtr& msg) {
//     local_fin = msg->data;
// }

// void CollisionCheck::traffic_sign_cb(const ::ConstPtr* msg) {}

void CollisionCheck::object_path_prediction(const std::vector<object>& objects) {
    for(int i = 0; i < objects.size(); i++) {
        float min_path_y = objects[i].min_y + 3 * v_y;
        float max_path_y = objects[i].max_y + 3 * v_y;
        float min_path_x = objects[i].min_x + 3 * v_x;
        float max_path_x = objects[i].max_x + 3 * v_x;
        objects_path.push_back(object(min_path_y, max_path_y, min_path_x, min_path_y, v_x, v_y));
    }
}

void CollisionCheck::collision_check(const int& next_rop, const int& speed, const std::vector<object>& objects_path) {
    size_t size = objects_path.size();
    if (next_rop == 4) {
        for(int i = 0; i < size; i++) {
            float leftob = objects[i].max_y;
            float rightob = objects[i].min_y;
            float frontob = objects[i].min_x;
            float vehicle_min_x = 3 * speed + 2.5; // interested in all objects of 3secs later

            if(((2.5 <= frontob && frontob <= vehicle_min_x) && (-1.5 <= rightob && rightob <= 1.5)) ||
                ((2.5 <= frontob && frontob <= vehicle_min_x) && (-1.5 <= leftob && leftob <= 1.5))) {
                maybe.push_back(objects[i]);
            }
        }
    } else if (next_rop == 3) {

    } else if (next_rop == 5) {
        for(int i = 0; i < size; i++) {
            float leftob = objects[i].max_y;
            float rightob = objects[i].min_y;
            float frontob = objects[i].max_x;
            float vehicle_max_x = 3 * speed + 2.5; // interested in all objects of 3secs later

            if(((2.5 <= frontob && frontob <= vehicle_max_x) && (-1.5 <= rightob && rightob <= 1.5)) ||
                ((2.5 <= frontob && frontob <= vehicle_max_x) && (-1.5 <= leftob && leftob <= 1.5))) {
                maybe.push_back(objects[i]);
            }
        } 
    }


void CollisionCheck::right_turn(const int& next_rop, const int& next2_rop)  {
    if (next_rop == 4 && next2_rop == 2) {
        jcic_stop = true;
        if(trafficlight_none || green_light) {
            jcic_stop = false;
        }
    }
}

void CollisionCheck::ACC(const int& next_rop, const int& speed, const std::vector<object>& objects) { // knowing speed of head vehicle, speed info
    if (nextrop == 3 || next_rop == 4) {
        for (int i = 0; i < objects.size(); i++) {
            float leftbd = object[i].min_y;
            float rightbd = object[i].max_y;
            float bumper = object[i].min_x;
            float ACC_x = 3 * speed + 2.5;
            float AEB_x = 1.5 * speed + 2.5;
            float 
            if (-1.5 <= leftob && 1.5 <= rightob) {
                if (AEB_x <= bumper && bumper <= ACC_x) {
                    acc = true;
                } else if (ACC_x <= bumper) {
                    acc = false;
                }
            }
        }
    }
}

void CollisionCheck::AEB(const int& next_rop, const int& speed, const std::vector<object>& maybe) {
    path_
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

#include "behavior_planner.hpp"

BehaviorPlanner::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0), v_y(0.0), v_x(0.0) {}
BehaviorPlanner::object::object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x) : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x), v_y(v_y), v_x(v_x) {}

BehaviorPlanner::BehaviorPlanner() {
    object_sub = nh.subscribe("/bounding_box", 10, &BehaviorPlanner::object_cb, this);
    // speed_sub = nh.subscribe("/carla/hero/Speed", 10, &CollisionCheck::speed_cb, this);
    // yaw_sub = nh.subscribe("/carla/hero/localization", 10, &CollisionCheck::yaw_cb, this);
    // steering_angle_sub = nh.subscribe("")
    // road_option_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &CollisionCheck::road_option_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &BehaviorPlanner::speed_cb, this);
    // local_fin_sub = nh.subscribe("/carla/hero/local_fin", 10, &CollisionCheck::local_fin_cb, this);
    // traffic_sign_sub = nh.subscrbie("/carla/traffic_sign"), 10, &CollisionCheck::traffic_sign_cb, this);
    AEB_pub = nh.advertise<std_msgs::Bool>("/carla/hero/AEB", 10);
    // local_req_pub = nh.advertise<std_msgs::Bool>("/localplan_request", 10);
    // acc_speed_pub = nh.advertise<carla_msgs::CarlaSpeedometer>("/localplan_request", 10);
    objects.reserve(500);
    // maybe.reserve(50);
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
    // object_prediction(objects);
}

void BehaviorPlanner::collision_check(const std::vector<object>& objects) {
    size_t size = objects.size();
    bool AEB_loop;
    for (int i = 0; i < size; i++) {
        float min_x = objects[i].min_x;
        float min_y = objects[i].min_y;
        float max_y = objects[i].max_y;
        if ((2.9 < min_x && min_x < 2.9 + 1.5 * speed) &&
        (((max_y >= -1) && (min_y < -1)) || ((min_y < 1) && (max_y > 1)) || (-1 < min_y && max_y < 1))) {
            AEB = true;
            return;
        } else {
            AEB_loop = false;
        }
    }
    AEB = AEB_loop;
}

// void CollisionCheck::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
//     pose[0] = msg->x;
//     pose[1] = msg->y;
// }
// void CollisionCheck::road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg) { //necessary?
//     next_rop = msg->road_options[0];
//     next2_rop = msg->road_options[1];
// }
void BehaviorPlanner::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
}
// void CollisionCheck::local_fin_cb(const std_msgs::Bool::ConstPtr& msg) {
//     local_fin = msg->data;
// }

// void CollisionCheck::traffic_sign_cb(const ::ConstPtr* msg) {}

// void CollisionCheck::object_prediction(const std::vector<object>& objects) {
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

// void CollisionCheck::collision_check(const int& next_rop, const int& speed, const std::vector<object>& objects_path) {
//     size_t size = objects_path.size();
//     if (next_rop == 4) {
//         for(int i = 0; i < size; i++) {
//             float leftob = objects[i].max_y;
//             float rightob = objects[i].min_y;
//             float frontob = objects[i].min_x;
//             float vehicle_min_x = 3 * speed + 2.5; // interested in all objects of 3secs later

//             if(((2.5 <= frontob && frontob <= vehicle_min_x) && (-1.5 <= rightob && rightob <= 1.5)) ||
//                 ((2.5 <= frontob && frontob <= vehicle_min_x) && (-1.5 <= leftob && leftob <= 1.5))) {
//                 maybe.push_back(objects[i]);
//             }
//         }
//     } else if (next_rop == 3) {

//     } else if (next_rop == 5) {
//         for(int i = 0; i < size; i++) {
//             float leftob = objects[i].max_y;
//             float rightob = objects[i].min_y;
//             float frontob = objects[i].max_x;
//             float vehicle_max_x = 3 * speed + 2.5; // interested in all objects of 3secs later

//             if(((2.5 <= frontob && frontob <= vehicle_max_x) && (-1.5 <= rightob && rightob <= 1.5)) ||
//                 ((2.5 <= frontob && frontob <= vehicle_max_x) && (-1.5 <= leftob && leftob <= 1.5))) {
//                 maybe.push_back(objects[i]);
//             }
//         } 
//     }


// void CollisionCheck::right_turn(const int& next_rop, const int& next2_rop)  {
//     if (next_rop == 4 && next2_rop == 2) {
//         jcic_stop = true;
//         if(trafficlight_none || green_light) {
//             jcic_stop = false;
//         }
//     }
// }

// void CollisionCheck::ACC(const int& next_rop, const int& speed, const std::vector<object>& objects) { // knowing speed of head vehicle, speed info
//     if (nextrop == 3 || next_rop == 4) {
//         for (int i = 0; i < objects.size(); i++) {
//             float leftbd = object[i].min_y;
//             float rightbd = object[i].max_y;
//             float bumper = object[i].min_x;
//             float ACC_x = 3 * speed + 2.5;
//             float AEB_x = 1.5 * speed + 2.5;
//             float 
//             if (-1.5 <= leftob && 1.5 <= rightob) {
//                 if (AEB_x <= bumper && bumper <= ACC_x) {
//                     acc = true;
//                 } else if (ACC_x <= bumper) {
//                     acc = false;
//                 }
//             }
//         }
//     }
// }

// void CollisionCheck::AEB(const int& next_rop, const int& speed, const std::vector<object>& maybe) {
//     path_
// }

// void CollisionCheck::stop_check() {
//     if (local_plan == false) {
//         object_detect(next_rop, speed, objects);
//         junction_intersection(next_rop, next2_rop);
//         if (object_detected || jcic_stop) {
//             g_stop = true;
//         } else {
//             g_stop = false;
//         }
//     } else {
//         object_detected = false;
//         jcic_stop = false;
//         trafficlight_none = true;
//         green_light = false;
//         g_stop = false;
//     }
//     std::cout << "stop checking" << std::endl;
// }

void BehaviorPlanner::publisher() {
    collision_check(objects);
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
