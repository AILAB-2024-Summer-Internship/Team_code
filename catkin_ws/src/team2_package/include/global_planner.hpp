#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "team2_package/vehicle_state.h"
#include "team2_package/globalwaypoints.h"

class GlobalPlanner {
public:
    struct waypoint {
        float x;
        float y;
        
        waypoint();
        waypoint(float x, float y);
    };

    GlobalPlanner();

    void load_csv(const std::string& filename);
    void path_publisher();

private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;

    std::vector<float> pose;
    std::vector<int> options;
    std::vector<waypoint> waypoints;

    int idx = 0;
    int endidx = 10;
    const int SIZE = 10000;
};

#endif
