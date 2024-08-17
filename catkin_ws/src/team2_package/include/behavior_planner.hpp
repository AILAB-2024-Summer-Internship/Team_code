#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <deque>
#include "vision_msgs/BoundingBox2DArray.h" // prediction
#include "team2_package/globalwaypoints.h" // collision check
#include "carla_msgs/CarlaSpeedometer.h" 
#include "team2_package/vehicle_state.h" // AEB
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h" //ACC
#include <iostream>

class BehaviorPlanner{
  public:
    struct object{ // vertical y, horizontal x 
        float min_y;
        float min_x;
        float max_y;
        float max_x;
        float v_y;
        float v_x;

        object();
        object(float min_y, float min_x, float max_y, float max_x, float v_y, float v_x);
    };

    struct waypoint {
      float x;
      float y;
      float speed;
      
      waypoint();
      waypoint(float x, float y, float speed);
    };

    // struct waypoint_pub {
    //   float x;
    //   float y;

    //   waypoint_pub();
    //   waypoint_pub(float x, float y);
    // };

    BehaviorPlanner();

    void object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg);
    void waypoint_cb(const team2_package::globalwaypoints::ConstPtr& msg);
    void pose_cb(const team2_package::vehicle_state::ConstPtr& msg);
    void speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg);
    //void yolo_cb();

    void object_prediction(const std::vector<object>& objects);
    void ego_prediction(const std::vector<waypoint>& waypoints, const std::vector<float>& pose, const float& speed);
    void collision_check(const std::vector<std::vector<object>>& objects_predict_3s, const std::vector<object>& ego_predict_3s);
    // void speed_profiling(const std::vector<waypoint>& waypoints_conv, const int& road_option, const std::vector<object>& objects);
    // void local_planner(const int& road_option, const std::vector<object>& objects, const float& speed, const bool& AEB);
    void publisher();

  private:
    ros::NodeHandle nh;
    ros::Subscriber object_sub;
    ros::Subscriber waypoint_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber pose_sub;
    // ros::Subscriber yolo_sub;

    ros::Publisher AEB_pub;
    // ros::Publisher ref_speed_pub;
    // ros::Publisher ACC_pub;
    // ros::Publisher distance_pub;
    // ros::Publisher waypoints_pub;

    ros::Timer timer;

    std::vector<object> objects;
    std::vector<waypoint> waypoints;
    std::vector<waypoint> waypoints_conv;
    std::vector<float> pose;
    std::vector<std::vector<object>> objects_predict_3s;
    std::vector<object> ego_predict_3s;
    // std::vector<waypoint_pub> global_waypoints;
    // std::vector<waypoint_pub> local_waypoints;

    int road_option;
    float speed;
    bool AEB = false;
    bool slow_down = false;
    // float ref_speed;
    // bool ACC = false;
    // float distance;
    // bool local_planning= false;
};
