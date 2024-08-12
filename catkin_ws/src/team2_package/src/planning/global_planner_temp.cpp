#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <carla_msgs/CarlaRoute.h>

struct waypoint {
    float x;
    float y;

    waypoint(float x, float y) : x(x), y(y) {}
};

class GlobalPlanner {
public:
    GlobalPlanner();
    void route_cb(const carla_msgs::CarlaRoute::ConstPtr& msg);
    void route_interpolate(const std::vector<waypoint>& waypoints, const std::vector<int>& options);
    void save_to_csv(const std::string& filename);

private:
    ros::NodeHandle nh;
    ros::Subscriber route_sub;
    std::vector<int> options;
    std::vector<waypoint> waypoints;
    std::vector<int> my_options;
    std::vector<waypoint> my_waypoints;

    const int RESERVE_SIZE = 10000;
    const int MY_SIZE = 10000;
};

GlobalPlanner::GlobalPlanner() {
    route_sub = nh.subscribe("/carla/hero/global_plan", 10, &GlobalPlanner::route_cb, this);
    options.reserve(RESERVE_SIZE);
    waypoints.reserve(RESERVE_SIZE);
    my_options.reserve(MY_SIZE);
    my_waypoints.reserve(MY_SIZE);
}

void GlobalPlanner::route_cb(const carla_msgs::CarlaRoute::ConstPtr& msg) {
    size_t size = msg->road_options.size();
    for (int i = 0; i < size; ++i) {
        options.emplace_back(msg->road_options[i]);
        waypoints.emplace_back(waypoint(static_cast<float>(msg->poses[i].position.x), static_cast<float>(msg->poses[i].position.y)));
    }
    route_interpolate(waypoints, options);
    save_to_csv("/home/ailab/Desktop/team_code/global_waypoints_team2.csv"); // 데이터가 갱신될 때마다 저장
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
        if ((x1 == x2) && (y1 == y2)) {
            i++;
        } else {
            if (option == 3 || option == 4) {
                int num_points = static_cast<int>((std::sqrt(dx * dx + dy * dy)) / 2);
                if (num_points > 1) {
                    for (int j = 0; j < num_points; j++) {
                        my_options.push_back(option);
                        float t = static_cast<float>(j) / num_points;
                        float ix = x1 + t * dx;
                        float iy = y1 + t * dy;
                        my_waypoints.push_back(waypoint(ix, iy));
                    }
                } else {
                    my_options.push_back(option);
                    my_waypoints.push_back(waypoint(x1, y1));
                }
            } else {
                my_options.push_back(option);
                my_waypoints.push_back(waypoint(x1, y1));
            }
        }
    }
}

void GlobalPlanner::save_to_csv(const std::string& filename) {
    std::ofstream file;
    file.open(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing." << std::endl;
        return;
    }

    // CSV 헤더 작성
    file << "Option, X, Y\n";

    // my_options 및 my_waypoints 벡터의 데이터를 CSV로 저장
    for (size_t i = 0; i < my_waypoints.size(); ++i) {
        file << my_options[i] << ", " << my_waypoints[i].x << ", " << my_waypoints[i].y << "\n";
    }

    file.close();
    std::cout << "Data successfully saved to " << filename << std::endl;
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "global_planner");

    // GlobalPlanner 객체 생성
    GlobalPlanner planner;

    // ROS 루프 시작
    ros::spin();

    return 0;
}
