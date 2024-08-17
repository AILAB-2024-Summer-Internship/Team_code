#include "behavior_planner.hpp"

BehaviorPlanner::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0), v_y(0.0), v_x(0.0) {}
BehaviorPlanner::object::object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x) 
    : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x), v_y(v_y), v_x(v_x) {}

BehaviorPlanner::waypoint::waypoint() : x(0.0), y(0.0), speed(0.0) {}
BehaviorPlanner::waypoint::waypoint(float x, float y, float speed) : x(x), y(y), speed(speed) {}

// BehaviorPlanner::waypoint_pub::waypoint_pub() : x(0.0), y(0.0) {}
// BehaviorPlanner::waypoint_pub::waypoint_pub(float x, float y) : x(x), y(y) {}

BehaviorPlanner::BehaviorPlanner() {
    object_sub = nh.subscribe("/pub_planner_bbox", 10, &BehaviorPlanner::object_cb, this);
    waypoint_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &BehaviorPlanner::waypoint_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &BehaviorPlanner::speed_cb, this);
    pose_sub = nh.subscribe("/carla/hero/localization", 10, &BehaviorPlanner::pose_cb, this);
    // yolo_sub = nh.subscribe("", 10, &BehaviorPlanner::yolo_cb, this);
    
    AEB_pub = nh.advertise<std_msgs::Bool>("/carla/hero/AEB", 10);
    // ref_speed_pub = nh.advertise<std_msgs::Float32>("/carla/hero/ref_speed", 10);
    // ACC_pub = nh.advertise<std_msgs::Bool>("/carla/hero/ACC", 10);
    // distance_pub = nh.advertise<std_msgs::Float32>("/carla/hero/ACC_dist", 10);
    // waypoints_pub = nh.advertise<team2_package::globalwaypoints>("waypoints", 10);

    timer = nh.createTimer(ros::Duration(0.3), &BehaviorPlanner::publisher, this);

    objects.reserve(500);
    pose.reserve(3);
    waypoints.reserve(10);
}

void BehaviorPlanner::object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg) {
    objects.clear();
    size_t size = msg->boxes.size();
    objects.reserve(size);
    for (size_t i = 0; i < size; i++) {
        float min_y = msg->boxes[i].center.y - (msg->boxes[i].size_y / 2);
        float min_x = msg->boxes[i].center.x - (msg->boxes[i].size_x / 2);
        float max_y = msg->boxes[i].center.y + (msg->boxes[i].size_y / 2);
        float max_x = msg->boxes[i].center.x + (msg->boxes[i].size_x / 2);
        objects.emplace_back(min_y, min_x, max_y, max_x, 0.0, 0.0);
    }
    object_prediction(objects);
    std::cout << "object size : " << size << std::endl;
}

void BehaviorPlanner::waypoint_cb(const team2_package::globalwaypoints::ConstPtr& msg) {
    waypoints.clear();
    size_t size = msg->road_options.size();
    waypoints.reserve(size);
    road_option = msg->road_options[0];
    for (size_t i = 0; i < size; i++) {
        waypoints.emplace_back(msg->x[i], msg->y[i], 0.0);
    }
    ego_prediction(waypoints, pose, speed);
}

void BehaviorPlanner::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
    pose.clear();
    pose.reserve(3);
    pose.push_back(msg->x);
    pose.push_back(msg->y);
    pose.push_back(msg->yaw);
}

void BehaviorPlanner::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
    std::cout << "speed: " << speed << std::endl;
}

void BehaviorPlanner::object_prediction(const std::vector<object>& objects) {
    objects_predict_3s.clear();
    objects_predict_3s.reserve(objects.size());
    for (const auto& obj : objects) {
        std::vector<object> predict_3s;
        predict_3s.reserve(10);
        for (int j = 0; j < 10; j++) {
            float min_path_y = obj.min_y + 0.3 * j * obj.v_y;
            float max_path_y = obj.max_y + 0.3 * j * obj.v_y;
            float min_path_x = obj.min_x + 0.3 * j * obj.v_x;
            float max_path_x = obj.max_x + 0.3 * j * obj.v_x;
            predict_3s.emplace_back(min_path_y, min_path_x, max_path_y, max_path_x, 0.0, 0.0);
        }
        objects_predict_3s.emplace_back(predict_3s);
        std::cout << "predict size: " << predict_3s.size() << std::endl;
    }
    
    std::cout << "object_prediction size : " << objects_predict_3s.size() << std::endl;
}

void BehaviorPlanner::ego_prediction(const std::vector<waypoint>& waypoints, const std::vector<float>& pose, const float& speed) {
    waypoints_conv.clear();
    size_t size = waypoints.size();

    float theta = pose[2];
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    for (int i = 0; i < size - 2; i++) {
        float dx1 = waypoints[i].x - pose[0];
        float dy1 = waypoints[i].y - pose[1];
        float dx2 = waypoints[i + 1].x - pose[0];
        float dy2 = waypoints[i + 1].y - pose[1];
        float dx3 = waypoints[i + 2].x - pose[0];
        float dy3 = waypoints[i + 2].y - pose[1];

        float x1 = dx1 * cos_theta + dy1 * sin_theta;
        float y1 = -dx1 * sin_theta + dy1 * cos_theta;
        float x2 = dx2 * cos_theta + dy2 * sin_theta;
        float y2 = -dx2 * sin_theta + dy2 * cos_theta;
        float x3 = dx3 * cos_theta + dy3 * sin_theta;
        float y3 = -dx3 * sin_theta + dy3 * cos_theta;

        float dxdy1 = (x2 - x1) / (y2 - y1);
        float dxdy2 = (x3 - x2) / (y3 - y2);
        float curvature = (dxdy2 - dxdy1) / std::pow((1 + dxdy1 * dxdy1), 1.5);

        float max_speed = (curvature < 0.000001) ? std::sqrt(0.5 * 9.81 * (1 / curvature)) : 10;

        waypoints_conv.push_back(waypoint(x1, y1, max_speed));
        if (i == 7) {
            waypoints_conv.push_back(waypoint(x2, y2, max_speed));
            waypoints_conv.push_back(waypoint(x3, y3, max_speed));
        }
    }
    std::cout << "waypoints_conv size: " << waypoints_conv.size() << std::endl;
    ego_predict_3s.clear();
    ego_predict_3s.reserve(10);

    float car_half_width = 0.95;
    float car_half_length = 2.48;

    const auto& waypoint0 = waypoints_conv[0];
    float distance0 = std::sqrt(waypoint0.x * waypoint0.x + waypoint0.y * waypoint0.y);
    int cnt = 10;
    std::cout << "speed2: " << speed << std::endl;
    std::cout << "distance_0: " << distance0 << std::endl;
    if (speed > 2) {
        int cnt0 = static_cast<int>(distance0 / speed * 0.3);

        for (int n = 0; n < cnt0; n++) {
            float offset = speed * 0.3 * n;
            float min_x = offset - car_half_length;
            float max_x = offset + car_half_length;

            ego_predict_3s.emplace_back(-car_half_width, min_x, car_half_width, max_x, 0.0, 0.0);

            if (--cnt < 1) {
                return;
            }
        }

        for (size_t i = 1; i < waypoints_conv.size() - 1; i++) {
            const auto& waypoint1 = waypoints_conv[i];
            const auto& waypoint2 = waypoints_conv[i + 1];
            float dx = waypoint2.x - waypoint1.x;
            float dy = waypoint2.y - waypoint1.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            float angle = atan2(dy, dx) - M_PI_2;
            float cos_angle = cos(angle);
            float sin_angle = sin(angle);

            float actual_speed = (waypoint1.speed > speed) ? speed : waypoint1.speed;
            int cnt00 = static_cast<int>(distance / actual_speed * 0.3);

            for (int j = 0; j < cnt00; j++) {
                float offset_x = (dx / distance0) * actual_speed * 0.3 * j + waypoint1.x;
                float offset_y = (dy / distance0) * actual_speed * 0.3 * j + waypoint1.y;

                float min_y = offset_y - car_half_width;
                float min_x = offset_x - car_half_length;
                float max_y = offset_y + car_half_width;
                float max_x = offset_x + car_half_length;

                float r_min_y = min_y * cos_angle - min_x * sin_angle;
                float r_min_x = min_y * sin_angle + min_x * cos_angle;
                float r_max_y = max_y * cos_angle - max_x * sin_angle;
                float r_max_x = max_y * sin_angle + max_x * cos_angle;

                ego_predict_3s.emplace_back(r_min_y, r_min_x, r_max_y, r_max_x, 0.0, 0.0);

                if (--cnt < 1) {
                    return;
                }
            }
        }
    } else {
        int cnt0 = static_cast<int>(distance0 / 2 * 0.3);

        for (int n = 0; n < cnt0; n++) {
            float offset = 2 * 0.3 * n;
            float min_x = offset - car_half_length;
            float max_x = offset + car_half_length;

            ego_predict_3s.emplace_back(-car_half_width, min_x, car_half_width, max_x, 0.0, 0.0);

            if (--cnt < 1) {
                return;
            }
        }

        for (size_t i = 1; i < waypoints_conv.size() - 1; i++) {
            const auto& waypoint1 = waypoints_conv[i];
            const auto& waypoint2 = waypoints_conv[i + 1];
            float dx = waypoint2.x - waypoint1.x;
            float dy = waypoint2.y - waypoint1.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            float angle = atan2(dy, dx) - M_PI_2;
            float cos_angle = cos(angle);
            float sin_angle = sin(angle);

            int cnt00 = static_cast<int>(distance / 2 * 0.3);

            for (int j = 0; j < cnt00; j++) {
                float offset_x = (dx / distance0) * 2 * 0.3 * j + waypoint1.x;
                float offset_y = (dy / distance0) * 2 * 0.3 * j + waypoint1.y;

                float min_y = offset_y - car_half_width;
                float min_x = offset_x - car_half_length;
                float max_y = offset_y + car_half_width;
                float max_x = offset_x + car_half_length;

                float r_min_y = min_y * cos_angle - min_x * sin_angle;
                float r_min_x = min_y * sin_angle + min_x * cos_angle;
                float r_max_y = max_y * cos_angle - max_x * sin_angle;
                float r_max_x = max_y * sin_angle + max_x * cos_angle;

                ego_predict_3s.emplace_back(r_min_y, r_min_x, r_max_y, r_max_x, 0.0, 0.0);

                if (--cnt < 1) {
                    return;
                }
            }
        }
    }
    std::cout << "ego_predict size: " << ego_predict_3s.size() << std::endl;
}

void BehaviorPlanner::collision_check(const std::vector<std::vector<object>>& objects_predict_3s, const std::vector<object>& ego_predict_3s) {
    bool brake = false;
    bool slow_down_loop = false;
    
    for (size_t i = 0; i < objects_predict_3s.size(); i++) {
        for (int j = 0; j < ego_predict_3s.size(); j++) {
            if ((((ego_predict_3s[j].max_x > objects_predict_3s[i][j].min_x) && (ego_predict_3s[j].max_x < objects_predict_3s[i][j].max_x)) 
                || ((ego_predict_3s[j].min_x < objects_predict_3s[i][j].max_x) && (ego_predict_3s[j].min_x > objects_predict_3s[i][j].min_x))) &&
                (((ego_predict_3s[j].max_y > objects_predict_3s[i][j].min_y) && (ego_predict_3s[j].max_y < objects_predict_3s[i][j].max_y)) 
                || ((ego_predict_3s[j].min_y < objects_predict_3s[i][j].max_y) && (ego_predict_3s[j].min_y > objects_predict_3s[i][j].min_y)))) {
                    if (j <= 5) {
                        brake = true;
                        break;
                    } else if (j > 5) {
                        slow_down_loop = true;
                        break;
                    }
                }
        }
        if (brake || slow_down_loop) break;
    }
    std::cout << "brake: " << brake << std::endl;
    AEB = brake;
    slow_down = slow_down_loop;
}

// void BehaviorPlanner::speed_profiling(const std::vector<waypoint>& waypoints_conv, const int& road_option, const std::vector<object>& objects) { // trafficlight 추가
//     if (slow_down) {
//         ref_speed = 5.0;
//     } else {
//         const auto& waypoint1 = waypoints_conv[0];
//         const auto& waypoint2 = waypoints_conv[1];
//         float speed1 = waypoint1.speed;
//         float speed2 = waypoint2.speed;
//         std::deque<float> ref_speed_deq;
//         if(ref_speed_deq.empty()) {
//             ref_speed_deq.push_back(3);
//             ref_speed_deq.push_back(speed1);
//             ref_speed_deq.push_back(speed2);
//         }
//         if(ref_speed_deq[1] != speed1 || ref_speed_deq[2] != speed2) {
//             ref_speed_deq.pop_front();
//             ref_speed_deq.push_back(speed2);
//         }
//         ref_speed = ref_speed_deq[0];
//     }
//     if (road_option == 4) {
//         size_t size = objects.size();
//         bool ACC_loop = false;
//         for (int i = 0; i < size; i++) {
//             float min_x = objects[i].min_x;
//             float min_y = objects[i].min_y;
//             float max_y = objects[i].max_y;
//             if ((2.50 < min_x && min_x < 2.7 + 3 * speed) &&
//             (-1.5 < min_y && max_y < 1.5)) {
//                 ACC_loop = true;
//                 distance = min_x - 2.55;
//                 break;
//             }
//         }
//         if (ACC_loop) {
//             ACC = true;
//         } else if (ACC && !ACC_loop) {
//             ACC = false;
//         }
//     }
// }

// void BehaviorPlanner::local_planner(const int& road_option, const std::vector<object>& objects, const float& speed, const bool& AEB, const std::vector<float>& pose) {
//     if (road_option == 4) {
//         size_t size = objects.size();
//         for (int i = 0; i < size; i++) {
//             float min_x = objects[i].min_x;
//             float min_y = objects[i].min_y;
//             float max_y = objects[i].max_y;
//             float v_x = objects[i].v_x;
//             if ((AEB == true) && (v_x == 0) && (1.5 > min_y && max_y > 1.5)) {
//                 if (pose[2]
//             }
//         }
//     }
// }

void BehaviorPlanner::publisher(const ros::TimerEvent& event) {
    collision_check(objects_predict_3s, ego_predict_3s);
    
    std_msgs::Bool msg;
    msg.data = AEB;
    // std_msgs::Bool msg2;
    // msg2.data = ACC;
    // if (local_planning) {
    //     waypoints_pub.publish(local_waypoints);
    // } else {
    //     waypoints_pub.publish(global_waypoints);
    // }

    AEB_pub.publish(msg);
    // ACC_pub.publish(msg2);
    // ref_speed_pub.publish(speed);
    // distance_pub.publish(distance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_planner");
    BehaviorPlanner behaviorplanner;
    ros::spin();
    return 0;
}
