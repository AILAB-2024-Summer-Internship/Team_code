#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "vision_msgs/BoundingBox2DArray.h" // prediction
#include "team2_package/globalwaypoints.h" // collision check
#include "carla_msgs/CarlaSpeedometer.h" 
#include "team2_package/vehicle_state.h"
#include "std_msgs/Bool.h" // AEB

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

    BehaviorPlanner();

    void object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg);
    // void waypoint_cb(const team2_package::globalwaypoints::ConstPtr& msg);
    void speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg);
    // void pose_cb(const team2_package::vehicle_state::ConstPtr& msg);

    // void prediction(const std::vector<object>& objects);
    // void collision_check(const std::vector<object>& objects, const std::vector<float>& pose, const int& speed, const std::vector<waypoint>& waypoints);
    void collision_check(const std::vector<object>& objects);
    void publisher();

  private:
    ros::NodeHandle nh;
    ros::Subscriber object_sub;
    ros::Subscriber waypoint_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber pose_sub;

    ros::Publisher AEB_pub;

    std::vector<object> objects;
    std::vector<waypoint> waypoints;
    std::vector<float> pose;
    std::vector<object> objects_predict_3s;
    std::vector<waypoint> waypoints_conv;
    std::vector<object> ego_predict_3s;

    float speed;
    bool AEB;
    int road_option;
};
