#include "global_planner.hpp"

GlobalPlanner::waypoint::waypoint() : x(0.0), y(0.0) {}
GlobalPlanner::waypoint::waypoint(float x, float y) : x(x), y(y) {}

GlobalPlanner::GlobalPlanner() {
    path_pub = nh.advertise<team2_package::globalwaypoints>("/carla/hero/my_global_plan", 10);

    pose.resize(2);
    options.reserve(SIZE);
    waypoints.reserve(SIZE);
    load_csv("global_waypoints_team2.csv")
}

void GlobalPlanner::pose_cb(const team2_package::vehicle_state::ConstPtr& msg) {
    pose[0] = msg->x;
    pose[1] = msg->y;
}

void GlobalPlanner::load_csv(const std::string& filename) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        
        int option;
        float x, y;

        std::getline(ss, value, ',');
        option = std::stoi(value);

        std::getline(ss, value, ',');
        x = std::stof(value);

        std::getline(ss, value, ',');
        y = std::stof(value);

        options.push_back(option);
        waypoints.emplace_back(waypoint(x, y));
    }

    file.close();
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
