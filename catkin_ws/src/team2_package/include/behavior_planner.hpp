#include <ros/ros.h>
#include <vector>
#include "carla_msgs/CarlaSpeedometer.h"
#include "vision_msgs/BoundingBox2DArray.h"
#include "team2_package/globalwaypoints.h"
#include "std_msgs/Bool.h"

class CollisionCheck{
  public:
    struct object{ // vertical y, horizontal x 
        float min_y;
        float min_x;
        float max_y;
        float max_x;

        object();
        object(float min_y, float min_x, float max_y, float max_x);
    };

    int next_rop;
    int next2_rop;
    float speed;
    bool object_detected = false;
    bool jcic_stop = false;
    bool trafficlight_none = true;
    bool green_light = true;
    bool local_plan = false;
    bool g_stop = false;

    CollisionCheck();

    void object_cb(const vision_msgs::BoundingBox2DArray::ConstPtr& msg);
    void road_option_cb(const team2_package::globalwaypoints::ConstPtr& msg);
    void speed_cb(const carla_msgs::CarlaSpeedometer::ConstPtr& msg);
    void localplan_cb(const std_msgs::Bool::ConstPtr& msg);
    // void traffic_sign_cb(const ::ConstPtr& msg);

    void object_detect(const int& next_rop, const int& speed, const std::vector<object>& objects);
    void junction_intersection(const int& next_rop, const int& next2_rop);
    void stop_check();
    void global_stop_publisher();

  private:
    ros::NodeHandle nh;
    ros::Subscriber object_sub;
    ros::Subscriber road_option_sub;
    ros::Subscriber speed_sub;
    ros::Publisher stop_pub;
    // ros::Publisher activate_lp_pub;

    std::vector<object> objects;
};