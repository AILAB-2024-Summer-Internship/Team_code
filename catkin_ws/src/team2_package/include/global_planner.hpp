#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "spline.h"
#include <tf/transform_datatypes.h>
#include <carla_msgs/CarlaRoute.h>
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

    int idx = 0;
    int endidx = 10;

    GlobalPlanner();

    void pose_cb(const team2_package::vehicle_state::ConstPtr& msg);
    void route_cb(const carla_msgs::CarlaRoute::ConstPtr& msg);
    void route_interpolate(const std::vector<waypoint>& waypoints, const std::vector<int>& options);
    void path_publisher();

private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Subscriber route_sub;
    ros::Publisher path_pub;

    std::vector<float> pose;
    std::vector<int> options;
    std::vector<waypoint> waypoints;
    std::vector<int> my_options;
    std::vector<waypoint> my_waypoints;

    const int RESERVE_SIZE = 500;
    const int MY_SIZE = 15000;
};

#endif