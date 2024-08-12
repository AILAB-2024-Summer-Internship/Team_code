#include "global_planner.hpp"

GlobalPlanner::waypoint::waypoint() : x(0.0), y(0.0) {}
GlobalPlanner::waypoint::waypoint(float x, float y) : x(x), y(y) {}

GlobalPlanner::GlobalPlanner() {
    pose_sub = nh.subscribe("/carla/hero/localization", 10, &GlobalPlanner::pose_cb, this);
    route_sub = nh.subscribe("/carla/hero/global_plan", 10, &GlobalPlanner::route_cb, this);
    path_pub = nh.advertise<team2_package::globalwaypoints>("/carla/hero/my_global_plan", 10);

    pose.resize(2);
    options.reserve(RESERVE_SIZE);
    waypoints.reserve(RESERVE_SIZE);
    my_options.reserve(MY_SIZE);
    my_waypoints.reserve(MY_SIZE);
}

void GlobalPlanner::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
    pose[0] = msg->x;
    pose[1] = msg->y;
}

void GlobalPlanner::route_cb(const carla_msgs::CarlaRoute::ConstPtr& msg) {
    size_t size = msg->road_options.size();
    for (int i = 0; i < size; ++i) {
        options.emplace_back(msg->road_options[i]);
        waypoints.emplace_back(waypoint(static_cast<float>(msg->poses[i].position.x), static_cast<float>(msg->poses[i].position.y)));
    }
    // route_interpolate(waypoints, options);
}

void GlobalPlanner::route_interpolate(const std::vector<waypoint>& waypoints, const std::vector<int>& options) {
    if (waypoints.empty()) {
        std::cerr << "Error: waypoints empty." << std::endl;
        return;
    }
    const size_t waypoints_size = waypoints.size();

    for (size_t i = 0; i < waypoints_size - 1; i++) {
        const int& option = options[i];
        const auto& waypoint1 = waypoints[i];
        const auto& waypoint2 = waypoints[i + 1];
        float x1 = waypoint1.x;
        float x2 = waypoint2.x;
        float y1 = waypoint1.y;
        float y2 = waypoint2.y;
        float dx = x2 - x1;
        float dy = y2 - y1;

        if (option == 3 || option == 4) {
            if (abs(dx) >= 5 && abs(dy) >= 5) { // cubic interpolation for curved road
                std::vector<Eigen::Vector4f> coeff(3);
                std::vector<float> x, y;
                Eigen::Vector3f vdx, delta;
                Eigen::Vector4f d;
                x.reserve(4);
                y.reserve(4);
                const auto& waypoint3 = waypoints[i + 2];
                float x3 = waypoint3.x;
                float y3 = waypoint3.y;
                int vidx;

                if (abs(x3 - x2) >= 50 && abs(y3 - y2) >= 50) {
                    vidx = 0;
                    const auto& waypoint4 = waypoints[i + 3];
                    float x4 = waypoint4.x;
                    float y4 = waypoint4.y;
                    x.push_back(x1);
                    x.push_back(x2);
                    x.push_back(x3);
                    x.push_back(x4);
                    y.push_back(y1);
                    y.push_back(y2);
                    y.push_back(y3);
                    y.push_back(y4);

                    for (int i = 0; i < 3; ++i) {
                        vdx[i] = x[i + 1] - x[i];
                        delta[i] = (y[i + 1] - y[i]) / vdx[i];
                    }
                
                    d[0] = delta[0];
                    d[3] = delta[2];
                    for (int i = 1; i < 3; ++i) {
                        if (delta[i - 1] * delta[i] > 0) {
                            d[i] = 2 / (1 / delta[i - 1] + 1 / delta[i]);
                        } else {
                            d[i] = 0;
                        }
                    }

                    for (int i = 0; i < 3; ++i) {
                        float a = y[i];
                        float b = d[i];
                        float c = (3 * delta[i] - 2 * d[i] - d[i + 1]) / vdx[i];
                        float dd = (d[i] + d[i + 1] - 2 * delta[i]) / (vdx[i] * vdx[i]);
                        coeff[i] = Eigen::Vector4f(a, b, c, dd);
                    }
                }
                int num_points = static_cast<int>((std::sqrt(dx * dx + dy * dy)) / 4);
                for (int j = 0; j < num_points; j++) {
                    my_options.push_back(option);
                    float t = static_cast<float>(j) / num_points;
                    float ix = x1 + t * dx;
                    float iy = coeff[vidx][0] + coeff[vidx][1] * t * dx + coeff[vidx][2] * t * dx * t * dx + coeff[vidx][3] * t * dx * t * dx * t * dx;
                    my_waypoints.push_back(waypoint(ix, iy));
                }
                vidx++;
            } else { // linear interpolation for straight road
                int num_points = static_cast<int>((std::sqrt(dx * dx + dy * dy)) / 2);
                for (int j = 0; j < num_points; j++) {
                    my_options.push_back(option);
                    float t = static_cast<float>(j) / num_points;
                    float ix = x1 + t * dx;
                    float iy = y1 + t * dy;
                    my_waypoints.push_back(waypoint(ix, iy));
                }
            }
        } else if (option == 1 || option == 2) { // cubic spline interpolaiton for intersection
            float mid_point_x, mid_point_y;
            std::vector<double> tempX, tempY;
            tempX.reserve(3);
            tempY.reserve(3);
            
            if (option == 1 && (dx * dy < 0)) {
                mid_point_x = (x2 + x1) * 0.5;
                mid_point_y = y1 * (1 - std::sqrt(3) / 2) + y2 * std::sqrt(3) / 2;
            } else if (option == 1 && (dx * dy > 0)) {
                mid_point_x = (x2 + x1) * 0.5;
                mid_point_y = y2 * (1 - std::sqrt(3) / 2) + y1 * std::sqrt(3) / 2;
            } else if (option == 2 && (dx * dy > 0)) {
                mid_point_x = (x2 + x1) * 0.5;
                mid_point_y = y1 * (1 - std::sqrt(3) / 2) + y2 * std::sqrt(3) / 2;
            } else {
                mid_point_x = (x2 + x1) * 0.5;
                mid_point_y = y2 * (1 - std::sqrt(3) / 2) + y1 * std::sqrt(3) / 2;
            }
            if (x1 < x2) {
                tempX.push_back(x1);
                tempX.push_back(mid_point_x);
                tempX.push_back(x2);
                tempY.push_back(y1);
                tempY.push_back(mid_point_y);
                tempY.push_back(y2);
            } else {
                tempX.push_back(x2);
                tempX.push_back(mid_point_x);
                tempX.push_back(x1);
                tempY.push_back(y2);
                tempY.push_back(mid_point_y);
                tempY.push_back(y1);
            }

            tk::spline s(tempX, tempY);
            float xm_1 = (x1 + mid_point_x) / 2;
            float ym_1 = static_cast<float>(s(xm_1));
            float xm_2 = (x2 + mid_point_x) / 2;
            float ym_2 = static_cast<float>(s(xm_2));
            for (int j = 0; j < 4; j++) {
                my_options.push_back(option);
            }
            my_waypoints.push_back(waypoint(x1, y1));
            my_waypoints.push_back(waypoint(xm_1, ym_1));
            my_waypoints.push_back(waypoint(mid_point_x, mid_point_y));
            my_waypoints.push_back(waypoint(xm_2, ym_2));
        } else {
            my_options.push_back(option);
            my_waypoints.push_back(waypoint(x1, y1));
        }
    }
}

void GlobalPlanner::path_publisher() {
    team2_package::globalwaypoints msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";

    size_t size_t = options.size();

    if (idx < size_t) {
        const auto& waypoint1 = waypoints[idx];
        const auto& waypoint2 = waypoints[idx + 1];
        float x1 = waypoint1.x;
        float y1 = waypoint1.y;
        float x2 = waypoint2.x;
        float y2 = waypoint2.y;
        float dx = x1 - x2;
        float dy = y1 - y2;

        if (abs(dx) < (abs(dy))) {
            if ((y1 <= pose[1] && pose[1] <= y2) || (y2 <= pose[1] && pose[1] <= y1)) {
                idx++;
            }
        }
        else if (abs(dy) < (abs(dx))) {
            if ((x1 <= pose[0] && pose[0] <= x2) || (x2 <= pose[0] && pose[0] <= x1)) {
                idx++;
            }
        } else {
            if ((x1 <= pose[0] && pose[0] <= x2) && (y1 <= pose[1] && pose[1] <= y2) || 
                (x2 <= pose[0] && pose[0] <= x1) && (y2 <= pose[1] && pose[1] <= y1)) {
                idx++;
            }
        }
        if (idx > static_cast<int>(size_t) - 10 && idx < static_cast<int>(size_t)) {
            endidx = size_t - idx;
        }
    }

    for (int i = 0; i < endidx; i++) {
        msg.road_options.emplace_back(options[idx + i]);
        msg.x.emplace_back(waypoints[idx + i].x);
        msg.y.emplace_back(waypoints[idx + i].y);
    }
    path_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    GlobalPlanner globalplanner;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        globalplanner.path_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
