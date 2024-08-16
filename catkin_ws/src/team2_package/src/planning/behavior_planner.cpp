#include "behavior_planner.hpp"

BehaviorPlanner::object::object() : min_y(0.0), min_x(0.0), max_y(0.0), max_x(0.0), v_y(0.0), v_x(0.0) {}
BehaviorPlanner::object::object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x) : min_y(min_y), min_x(min_x), max_y(max_y), max_x(max_x), v_y(v_y), v_x(v_x) {}

BehaviorPlanner::waypoint::waypoint() : x(0.0), y(0.0), speed(0.0) {}
BehaviorPlanner::waypoint::waypoint(float x, float y, float speed) : x(x), y(y), speed(speed) {}

BehaviorPlanner::waypoint_pub::waypoint_pub() : x(0.0), y(0.0) {}
BehaviorPlanner::waypoint_pub::waypoint_pub(float x, float y) : x(x), y(y) {}


BehaviorPlanner::BehaviorPlanner() {
    object_sub = nh.subscribe("/bounding_box", 10, &BehaviorPlanner::object_cb, this);
    waypoint_sub = nh.subscribe("/carla/hero/my_global_plan", 10, &BehaviorPlanner::waypoint_cb, this);
    speed_sub = nh.subscribe("/carla/hero/Speed", 10, &BehaviorPlanner::speed_cb, this);
    pose_sub = nh.subscribe("/carla/hero/localization", 10, &BehaviorPlanner::pose_cb, this);
    // yolo_sub = nh.subscribe("", 10, &BehaviorPlanner::yolo_cb, this);
    
    AEB_pub = nh.advertise<std_msgs::Bool>("/carla/hero/AEB", 10);
    // ref_speed_pub = nh.advertise<std_msgs::Float32>("/carla/hero/ref_speed", 10);
    // ACC_pub = nh.advertise<std_msgs::Bool>("/carla/hero/ACC", 10);
    // distance_pub = nh.advertise<std_msgs::Float32>("/carla/hero/ACC_dist", 10);
    // waypoints_pub = nh.advertise<team2_package::globalwaypoints>("waypoints", 10);

    objects.reserve(500);
    pose.reserve(3);
    waypoints.reserve(10);
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
    object_prediction(objects);
    collision_check(objects_predict_3s, ego_predict_3s);
}

void BehaviorPlanner::waypoint_cb(const team2_package::globalwaypoints::ConstPtr& msg) {
    size_t size = msg->road_options.size();
    for (int i = 0; i < size; i++) {
        waypoints[i].x = msg->x[i];
        waypoints[i].y = msg->y[i];
        waypoints[i].speed = 0.0;
    }
    road_option = msg->road_options[0];
    ego_prediction(waypoints, pose, speed);
}

void BehaviorPlanner::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
    pose[0] = msg->x;
    pose[1] = msg->y;
    pose[2] = msg->yaw;
}

void BehaviorPlanner::speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    speed = msg->speed;
}

void BehaviorPlanner::object_prediction(const std::vector<object>& objects) {
    for(int i = 0; i < objects.size(); i++) {
        std::vector<object> predict_3s;
        predict_3s.reserve(10);
        for(int j = 0; j < 10; j++) {
            float min_path_y = objects[i].min_y + 0.3 * j * objects[i].v_y;
            float max_path_y = objects[i].max_y + 0.3 * j * objects[i].v_y;
            float min_path_x = objects[i].min_x + 0.3 * j * objects[i].v_x;
            float max_path_x = objects[i].max_x + 0.3 * j * objects[i].v_x;
            predict_3s.push_back(object(min_path_y, max_path_y, min_path_x, max_path_x, 0, 0));
        }
        objects_predict_3s.emplace_back(predict_3s);
    }
}

void BehaviorPlanner::ego_prediction(const std::vector<waypoint>& waypoints, const std::vector<float>& pose, const int& speed) {
    size_t size = waypoints.size();
    for (int i = 0; i < size - 2; i++) {
        float theta = pose[2];
        float dx1 = waypoints[i].x-pose[0];
        float dy1 = waypoints[i].y-pose[1];
        float dx2 = waypoints[i+1].x-pose[0];
        float dy2 = waypoints[i+1].y-pose[1];
        float dx3 = waypoints[i+2].x-pose[0];
        float dy3 = waypoints[i+2].y-pose[1];
        float x1 = dx1 * cos(theta) + dy1 * sin(theta);
        float y1 = -dx1 * sin(theta) + dy1 * cos(theta);
        float x2 = dx2 * cos(theta) + dy2 * sin(theta);
        float y2 = -dx2 * sin(theta) + dy2 * cos(theta);
        float x3 = dx3 * cos(theta) + dy3 * sin(theta);
        float y3 = -dx3 * sin(theta) + dy3 * cos(theta);
        float dxdy1 = (x2 - x1) / (y2 - y1);
        float dxdy2 = (x3 - x2) / (y3 - y2);
        float curvature = (dxdy2-dxdy1) / std::pow((1 + dxdy1*dxdy1),1.5);
        float max_speed;
        if (curvature < 0.000001) {
            max_speed = std::sqrt(0.5 * 9.81 * (1 / curvature));
        } else {
            max_speed = 10;
        }
        waypoints_conv.push_back(waypoint(x1,y1,max_speed));
        if (i == 7) {
            waypoints_conv.push_back(waypoint(x2,y2,max_speed));
            waypoints_conv.push_back(waypoint(x3,y3,max_speed));
        }
    }
    size_t size_conv = waypoints_conv.size();
    const auto& waypoint0 = waypoints_conv[0];
    float x0 = waypoint0.x;
    float y0 = waypoint0.y;
    float distance0 = std::sqrt(std::pow(x0,2) + std::pow(y0,2));
    int cnt = 10;
    int cnt0 = static_cast<int>(distance0 / speed * 0.3);
    for (int n = 0; n < cnt0; n++) {
        float min_y = y0 / distance0 * 0.3 * n - 0.95;
        float min_x = x0 / distance0 * 0.3 * n - 2.48;
        float max_y = y0 / distance0 * 0.3 * n + 0.95;
        float max_x = x0 / distance0 * 0.3 * n + 2.48;
        float v_y = 0.0;
        float v_x = 0.0;
        ego_predict_3s.push_back(object(min_y, min_x, max_y, max_x, v_y, v_x));
        cnt -= 1;
        if (cnt < 1) {
            return;
        }
    }
    for (int i = 0; i < size_conv; i++) {
        const auto& waypoint1 = waypoints_conv[i];
        const auto& waypoint2 = waypoints_conv[i+1];
        float x1 = waypoint1.x;
        float x2 = waypoint2.x;
        float y1 = waypoint1.y;
        float y2 = waypoint2.y;
        float dx = x2 - x1;
        float dy = y2 - y1;
        float max_speed1 = waypoint1.speed;
        float distance = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
        float theta = atanf(dx / dy) - 90;
        if (max_speed1 > speed && speed > 0) {
            int cnt00 = static_cast<int>(distance / speed * 0.3);
            for(int j = 0; j < cnt00; j++) {
                float min_y = dy / distance0 * 0.3 * j - 0.95 + y1;
                float min_x = dx / distance0 * 0.3 * j - 2.48 + x1;
                float max_y = dy / distance0 * 0.3 * j + 0.95 + y1;
                float max_x = dx / distance0 * 0.3 * j + 2.48 + y2;
                float r_min_y = min_y * cos(theta) - min_x * sin(theta);
                float r_min_x = max_y * sin(theta) + min_x * cos(theta);
                float r_max_y = max_y * cos(theta) - max_x * sin(theta);
                float r_max_x = min_y * sin(theta) + max_x * cos(theta); 
                float v_y = 0.0;
                float v_x = 0.0;
                ego_predict_3s.push_back(object(r_min_y, r_min_x, r_max_y, r_max_x, v_y, v_x));
                cnt -= 1;
                if (cnt < 1) {
                    return;
                }
            }
        } else {
            int cnt00 = static_cast<int>(distance / max_speed1 * 0.3);
            for(int j = 0; j < cnt00; j++) {
                float min_y = dy / distance0 * 0.3 * j - 0.95 + y1;
                float min_x = dx / distance0 * 0.3 * j - 2.48 + x1;
                float max_y = dy / distance0 * 0.3 * j + 0.95 + y1;
                float max_x = dx / distance0 * 0.3 * j + 2.48 + y2;
                float r_min_y = min_y * cos(theta) - min_x * sin(theta);
                float r_min_x = max_y * sin(theta) + min_x * cos(theta);
                float r_max_y = max_y * cos(theta) - max_x * sin(theta);
                float r_max_x = min_y * sin(theta) + max_x * cos(theta); 
                float v_y = 0.0;
                float v_x = 0.0;
                ego_predict_3s.push_back(object(r_min_y, r_min_x, r_max_y, r_max_x, v_y, v_x));
                cnt -= 1;
                if (cnt < 1) {
                    return;
                }
            }
        }
    }
}

void BehaviorPlanner::collision_check(const std::vector<std::vector<object>>& objects_predict_3s, const std::vector<object>& ego_predict_3s) {
    size_t size = objects_predict_3s.size();
    bool brake = false;
    bool slow_down_loop = false;
    for (int i = 0; i < size; i++) {
        for(int j = 0; j < 10; j++) {
            if ((((ego_predict_3s[j].max_x > objects_predict_3s[i][j].min_x) && (ego_predict_3s[j].max_x < objects_predict_3s[i][j].max_x)) 
                || ((ego_predict_3s[j].min_x < objects_predict_3s[i][j].max_x) && (ego_predict_3s[j].min_x > objects_predict_3s[i][j].min_x))) &&
                (((ego_predict_3s[j].max_y > objects_predict_3s[i][j].min_y) && (ego_predict_3s[j].max_y < objects_predict_3s[i][j].max_y)) 
                || ((ego_predict_3s[j].min_y < objects_predict_3s[i][j].max_y) && (ego_predict_3s[j].min_y > objects_predict_3s[i][j].min_y)))) {
                    if (i <= 5) {
                        brake = true;
                        break;
                    } else if (i > 5) {
                        slow_down_loop = true;
                        break;
                    }
                }
        }
    }
    if (brake) {
        AEB = true;
    } else if (AEB && !brake) {
        AEB = false;
    }
    if (slow_down_loop) {
        slow_down = true;
    } else if (slow_down && !slow_down_loop) {
        slow_down = false;
    }
    speed_profiling(waypoints_conv, road_option, objects);
}

void BehaviorPlanner::speed_profiling(const std::vector<waypoint>& waypoints_conv, const int& road_option, const std::vector<object>& objects) { // trafficlight 추가
    if (slow_down) {
        ref_speed = 5.0;
    } else {
        const auto& waypoint1 = waypoints_conv[0];
        const auto& waypoint2 = waypoints_conv[1];
        float speed1 = waypoint1.speed;
        float speed2 = waypoint2.speed;
        std::deque<float> ref_speed_deq;
        if(ref_speed_deq.empty()) {
            ref_speed_deq.push_back(3);
            ref_speed_deq.push_back(speed1);
            ref_speed_deq.push_back(speed2);
        }
        if(ref_speed_deq[1] != speed1 || ref_speed_deq[2] != speed2) {
            ref_speed_deq.pop_front();
            ref_speed_deq.push_back(speed2);
        }
        ref_speed = ref_speed_deq[0];
    }
    if (road_option == 4) {
        size_t size = objects.size();
        bool ACC_loop = false;
        for (int i = 0; i < size; i++) {
            float min_x = objects[i].min_x;
            float min_y = objects[i].min_y;
            float max_y = objects[i].max_y;
            if ((2.50 < min_x && min_x < 2.55 + 3 * speed) &&
            (-1.5 < min_y && max_y < 1.5)) {
                ACC_loop = true;
                distance = min_x - 2.55;
                break;
            }
        }
        if (ACC_loop) {
        ACC = true;
        } else if (ACC && !ACC_loop) {
            ACC = false;
        }
    }
}

// void BehaviorPlanner::local_planner(const int& road_option, const std::vector<object>& objects) {
//     if (road_option == 4) {
//         size_t size = objects.size();
//         for (int i = 0; i < size; i++) {
//             float min_x = objects[i].min_x;
//             float min_y = objects[i].min_y;
//             float max_y = objects[i].max_y;
//             float v_x = objects[i].v_x;
//             if ((2.50 < min_x && min_x < 2.55 + 3 * speed) &&
//             (v_x == 0) && (1.5 > min_y && max_y > 1.5)) {
                
//             }
//         }
//     }
// }

void BehaviorPlanner::publisher() {
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
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        behaviorplanner.publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
