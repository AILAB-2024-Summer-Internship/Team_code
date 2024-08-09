#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <tf/tf.h>
#include "team2_package/vehicle_state.h"
#include <math.h>
// opencv library for matrix computation and kalman filter
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include "team2_package/localization_perform_measure.h"
#include <array>
#include <boost/bind.hpp>

#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)
#define EARTH_RADIUS 6378137.0          // 지구 반지름 (미터)
#define XY_RATIO (6356752.314245/6378137.0)
#define F (1.0 / 298.257223563) // 지구 편평률


struct Wgs84toEnu{
    //const float a = 6378137.0;
    //const float b = 6356752.314245;
    //const float elipse = 1 -pow(b/a,2); // e^2
    const double e2 = F * (2 - F);
    const double ref_latitude = 0.0;
    const double ref_longitude = 0.0;
    const double ref_altitude = 0.0;
    // const double M = pow(a*b,2) / pow((pow(a*cos(ref_latitude_rad),2)+pow(b*sin(ref_latitude_rad),2)), 1.5);
    // const double N = pow(a,2) / sqrt(pow(a*cos(ref_latitude_rad),2)+pow(b*sin(ref_latitude_rad),2));

    std::array<double,3> gps_position;
    std::array<double,3> gps_to_UTM(double lat, double lon, double alt);

    //const std::array<double,3>ECEF_REF;

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

struct Imu_processor{
    static double time_interval;
    geometry_msgs::Vector3 orientation;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 linear_accel;
    void callback(const sensor_msgs::Imu::ConstPtr &inertial_meas);
};

class Localizer {
    private:
        ros::NodeHandle nh;
        ros::Subscriber imu_subscriber;
        ros::Subscriber gps_subscriber;

        ros::Subscriber gps_subscriber2;

        ros::Subscriber speed_subscriber;
        ros::Subscriber odom_subscriber;

        sensor_msgs::Imu inertial_meas;
        sensor_msgs::NavSatFix gps_meas;

        sensor_msgs::NavSatFix gps_for_perform_meas;
        nav_msgs::Odometry odom;

        team2_package::localization_perform_measure loc_perform_msg;
        ros::Time last_time;
        ros::Duration time_interval;

        friend struct Wgs84toEnu;
        friend struct Imu_processer;

        struct Wgs84toEnu wgs84_to_enu;
        struct Imu_processor imu_processor;

        double current_speed;

        cv::Mat measurement;
        cv::KalmanFilter EKF;

        team2_package::vehicle_state vehicle_localization;
        ros::Publisher localization_publisher;
        ros::Publisher localization_performance_measurement_publisher;

    public:
        Localizer();
        void speed_sub_callback(const carla_msgs::CarlaSpeedometer::ConstPtr &speed_msg);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        cv::Mat f();
        void dead_reckoning();
        void local_publish();
        void loc_perform_meas_publish();
};