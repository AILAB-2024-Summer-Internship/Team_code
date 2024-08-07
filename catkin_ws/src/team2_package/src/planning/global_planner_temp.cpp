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
    options.resize(size);
    waypoints.resize(size);
    route_interpolate(waypoints, options);
}

void GlobalPlanner::cubic_spline_interpolation(const std::vector<waypoint>& waypoints, const int& option, const int& i) {
    const auto& waypoint1 = waypoints[i];
    const auto& waypoint2 = waypoints[i + 1];
    float x1 = waypoint1.x;
    float x2 = waypoint2.x;
    float y1 = waypoint1.y;
    float y2 = waypoint2.y;
    float dx = x2 - x1;
    float dy = y2 - y1;

    float mid_point_x, mid_point_y;
    std::vector<double> tempX, tempY;

    if (option == 1 && dx * dy < 0) {
        mid_point_x = x2 * (1 - 1 / std::sqrt(2)) + x1 * 1 / std::sqrt(2);
        mid_point_y = y1 * (1 - 1 / std::sqrt(2)) + y2 * 1 / std::sqrt(2);
        if (x1 < x2) {
            tempX = {waypoints[i-1].x, waypoints[i].x, mid_point_x, waypoints[i+1].x, waypoints[i+2].x};
            tempY = {waypoints[i-1].y, waypoints[i].y, mid_point_y, waypoints[i+1].y, waypoints[i+2].y};
        } else {
            tempX = {waypoints[i+2].x, waypoints[i+1].x, mid_point_x, waypoints[i].x, waypoints[i-1].x};
            tempY = {waypoints[i+2].y, waypoints[i+1].y, mid_point_y, waypoints[i].y, waypoints[i-1].y};
        }
    } else if (option == 1 && dx * dy >= 0) {
        mid_point_x = x1 * (1 - 1 / std::sqrt(2)) + x2 * 1 / std::sqrt(2);
        mid_point_y = y2 * (1 - 1 / std::sqrt(2)) + y1 * 1 / std::sqrt(2);
        if (x1 < x2) {
            tempX = {waypoints[i-1].x, waypoints[i].x, mid_point_x, waypoints[i+1].x, waypoints[i+2].x};
            tempY = {waypoints[i-1].y, waypoints[i].y, mid_point_y, waypoints[i+1].y, waypoints[i+2].y};
        } else {
            tempX = {waypoints[i+2].x, waypoints[i+1].x, mid_point_x, waypoints[i].x, waypoints[i-1].x};
            tempY = {waypoints[i+2].y, waypoints[i+1].y, mid_point_y, waypoints[i].y, waypoints[i-1].y};
        }
    } else if (option == 2 && dx * dy > 0) {
        mid_point_x = x2 * (1 - 1 / std::sqrt(2)) + x1 * 1 / std::sqrt(2);
        mid_point_y = y1 * (1 - 1 / std::sqrt(2)) + y2 * 1 / std::sqrt(2);
        if (x1 < x2) {
            tempX = {waypoints[i-1].x, waypoints[i].x, mid_point_x, waypoints[i+1].x, waypoints[i+2].x};
            tempY = {waypoints[i-1].y, waypoints[i].y, mid_point_y, waypoints[i+1].y, waypoints[i+2].y};
        } else {
            tempX = {waypoints[i+2].x, waypoints[i+1].x, mid_point_x, waypoints[i].x, waypoints[i-1].x};
            tempY = {waypoints[i+2].y, waypoints[i+1].y, mid_point_y, waypoints[i].y, waypoints[i-1].y};
        }
    } else {
        mid_point_x = x1 * (1 - 1 / std::sqrt(2)) + x2 * 1 / std::sqrt(2);
        mid_point_y = y2 * (1 - 1 / std::sqrt(2)) + y1 * 1 / std::sqrt(2);
        if (x1 < x2) {
            tempX = {waypoints[i-1].x, waypoints[i].x, mid_point_x, waypoints[i+1].x, waypoints[i+2].x};
            tempY = {waypoints[i-1].y, waypoints[i].y, mid_point_y, waypoints[i+1].y, waypoints[i+2].y};
        } else {
            tempX = {waypoints[i+2].x, waypoints[i+1].x, mid_point_x, waypoints[i].x, waypoints[i-1].x};
            tempY = {waypoints[i+2].y, waypoints[i+1].y, mid_point_y, waypoints[i].y, waypoints[i-1].y};
        }
    }

    tk::spline s(tempX, tempY);
    float xm_1 = (waypoints[i].x + mid_point_x) / 2;
    float ym_1 = static_cast<float>(s(xm_1));
    float xm_2 = (waypoints[i+1].x + mid_point_x) / 2;
    float ym_2 = static_cast<float>(s(xm_2));

    for (int j = 0; j < 5; ++j) {
        my_options.push_back(option);
    }

    my_waypoints.push_back(waypoint(x1, y1));
    my_waypoints.push_back(waypoint(xm_1, ym_1));
    my_waypoints.push_back(waypoint(mid_point_x, mid_point_y));
    my_waypoints.push_back(waypoint(xm_2, ym_2));
    my_waypoints.push_back(waypoint(x2, y2));
}

void GlobalPlanner::route_interpolate(const std::vector<waypoint>& waypoints, const std::vector<int>& options) {
    const size_t waypoints_size = options.size();

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
            Eigen::Vector4f xv;

            if (abs(dx) >= 5 && abs(dy) >= 5) { // cubic interpolation for curved road
                if (i + 2 < waypoints_size && i + 3 < waypoints_size) {  // Bounds check
                    const auto& waypoint3 = waypoints[i + 2];
                    float x3 = waypoint3.x;
                    float y3 = waypoint3.y;

                    if (abs(x3 - x2) >= 50 && abs(y3 - y2) >= 50) {
                        Eigen::Matrix4f A;
                        Eigen::Vector4f b;

                        const auto& waypoint4 = waypoints[i + 3];
                        float x4 = waypoint4.x;
                        float y4 = waypoint4.y;

                        A << std::pow(x4, 3), std::pow(x4, 2), x4, 1,
                                std::pow(x3, 3), std::pow(x3, 2), x3, 1,
                                std::pow(x2, 3), std::pow(x2, 2), x2, 1,
                                std::pow(x1, 3), std::pow(x1, 2), x1, 1;
                        b << y4, y3, y2, y1;
                        xv = A.colPivHouseholderQr().solve(b);
                    }

                    int num_points = static_cast<int>((std::sqrt(dx * dx + dy * dy)) / 4);
                    for (int j = 0; j < num_points; ++j) {
                        my_options.push_back(option);
                        float t = static_cast<float>(j) / num_points;
                        float ix = x1 + t * dx;
                        float iy = xv[0] * std::pow(ix, 3) + xv[1] * std::pow(ix, 2) + xv[2] * ix + xv[3];
                        my_waypoints.push_back(waypoint(ix, iy));
                    }
                }
            } else { // linear interpolation for straight road
                int num_points = static_cast<int>((std::sqrt(dx * dx + dy * dy)) / 4);
                for (int j = 0; j < num_points; ++j) {
                    my_options.push_back(option);
                    float t = static_cast<float>(j) / num_points;
                    float ix = x1 + t * dx;
                    float iy = y1 + t * dy;
                    my_waypoints.push_back(waypoint(ix, iy));
                }
            }
        } else if (option == 1 || option == 2) { // cubic spline interpolaiton for intersection
            cubic_spline_interpolation(waypoints, option, i);
        } else {
            my_options.push_back(option);
            my_waypoints.push_back(waypoint(x1, y1));
        }
    }
    const size_t my_options_size = my_options.size();
    my_options.resize(my_options_size);
    my_waypoints.resize(my_options_size);
}

void GlobalPlanner::path_publisher() {
    team2_package::globalwaypoints msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";

    size_t size_t = my_options.size();

    if (idx < size_t) {
        const auto& waypoint1 = my_waypoints[idx];
        const auto& waypoint2 = my_waypoints[idx + 1];
      
        if (abs(waypoint1.x-waypoint2.x) < (abs(waypoint1.y-waypoint2.y))) {
            if ((waypoint1.y <= pose[1] && pose[1] <= waypoint2.y) || (waypoint2.y <= pose[1] && pose[1] <= waypoint1.y)) {
                idx++;
            }
        }
        else if (abs(waypoint1.y-waypoint2.y) < (abs(waypoint1.x-waypoint2.x))) {
            if ((waypoint1.x <= pose[0] && pose[0] <= waypoint2.x) || (waypoint2.x <= pose[0] && pose[0] <= waypoint1.x)) {
                idx++;
            }
        }
        else {
            if ((waypoint1.x <= pose[0] && pose[0] <= waypoint2.x) && (waypoint1.y <= pose[1] && pose[1] <= waypoint2.y) || 
                (waypoint2.x <= pose[0] && pose[0] <= waypoint1.x) && (waypoint2.y <= pose[1] && pose[1] <= waypoint1.y)) {
                idx++;
            }
        }

        if (idx > static_cast<int>(size_t) - 10 && idx < static_cast<int>(size_t)) {
            endidx = size_t - idx;
        }
    }

    for (int i = 0; i < endidx; i++) {
        msg.road_options.emplace_back(my_options[idx + i]);
        msg.x.emplace_back(my_waypoints[idx + i].x);
        msg.y.emplace_back(my_waypoints[idx + i].y);
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