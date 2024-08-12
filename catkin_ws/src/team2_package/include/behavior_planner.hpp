#include <ros/ros.h>
#include <vector>
#include "vision_msgs/BoundingBox2DArray.h"
#include "carla_msgs/CarlaSpeedometer.h"
#include "team2_package/vehicle_state.h"
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include "team2_package/globalwaypoints.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

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

    BehaviorPlanner();

    void object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg);
    void speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg);
    void yaw_cb(const team2_package::vehicle_state::ConstPtr& msg);
    void steering_angle_cb(const carla_msgs::CarlaEgoVehicleControl::ConstPtr& msg);
    void road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg);
    // void local_fin_cb(const std_msgs::Bool::ConstPtr& msg);
    // void traffic_sign_cb(const ::ConstPtr& msg);

    void prediction();
    void collision_check();
    void AEB();
    void avoidance();
    void speed_profile();
    void publisher();

  private:
    ros::NodeHandle nh;
    ros::Subscriber object_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber yaw_sub;
    ros::Subscriber steering_angle_sub;
    // ros::Subscriber local_fin_sub;
    // ros::Subscriber traffic_sign_sub;

    ros::Publisher AEB_pub;
    // ros::Publisher local_req_pub;
    ros::Publisher speed_pub;
    ros::Publisher ACC_pub;
    ros::Publisher distance_pub;

    std::vector<object> objects;
};
