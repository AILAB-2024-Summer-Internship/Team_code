#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <team2_package/longitudinal_controller.h>
#include <team2_package/vehicle_state.h>
#include <team2_package/globalwaypoints.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <numeric>
#include <math.h>
#include <string>

#define WHEELBASE (2.84988) // Lincoln MKZ 2020
#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)
#define MAX_STEER_ANGLE (1.221730351448059)
#define MAX_ACCELERATION (2.5)
#define BASE_LOOK_AHEAD_DISTANCE (20.0)
#define STEERING_THRESHOLD (3.0*DEG_TO_RAD)

class Controller;

class Longitudinal_controller {
public:
    struct ACC;
    void AEB_ACC_control(Controller& controller);
};

struct Longitudinal_controller::ACC {
    struct speed_controller;
    struct spacing_controller;
};

struct Longitudinal_controller::ACC::speed_controller {
    float error;
    const float p_gain = 0.6;
    const float i_gain = 0.03;
    const float d_gain = 0.1;
    const float time_interval = 0.01; //100Hz
    static int reverse_check_count;
    static float i_term;
    static float last_error;
    static float command;

    speed_controller() : error(0) {}
    void operator()(Controller& controller);
};

struct Longitudinal_controller::ACC::spacing_controller {
    float error;
    const float preceding_vehicle_length = 3.0; // temporary. We must find the exact value!
    const float p_gain = 0.5;
    const float d_gain = 0.01;
    const float desired_distance = 10; // 10m(temporary)
    const float time_interval = 0.01; // 100Hz

    static float last_error;
    static float command;

    spacing_controller() : error(0) {}
    void operator()(Controller& controller);
};

class Lateral_controller {
    private:
        const float ld_gain_a = static_cast<float>(pow(BASE_LOOK_AHEAD_DISTANCE,-2));
        const float ld_gain_b = 0.1;
        const float ld_gain_vel = 0.5;
        const float weighted_term = 0.5;
        float curvature;
        float look_ahead_distance;
        float alpha;
        cv::Mat least_squares_x;
        cv::Mat least_squares_y;
        cv::Mat coefficients; // least squares 결과 저장할 행렬
        team2_package::vehicle_state rear_wheel_position;

    public:
        Lateral_controller() : look_ahead_distance(0) {}
        void polyfit_waypoints(const Controller& controller);
        void calculate_curvature(const Controller& controller);
        void find_look_ahead_distance(const Controller& controller);
        double calculateDistance(const geometry_msgs::Point& rear_wheel_position, const geometry_msgs::Point& point);
        void pure_pursuit(Controller& controller);
};

class Controller{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber ego_vehicle_pose_subscriber;
        ros::Subscriber vel_ref_subscriber;
        ros::Subscriber actual_vel_subscriber;
        ros::Subscriber waypoints_subscriber;
        ros::Subscriber preceding_vehicle_dist_subscriber;
        ros::Subscriber control_flag_subscriber;
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        ros::Publisher control_cmd_publisher;

        team2_package::vehicle_state ego_vehicle_pose;
        float vel_ref;
        double actual_vel;
        team2_package::globalwaypoints waypoints;
        std::vector<geometry_msgs::Point> closest_point_finder;
        float preceding_vehicle_dist;

        friend class Longitudinal_controller;
        friend struct Longitudinal_controller::ACC::speed_controller;
        friend struct Longitudinal_controller::ACC::spacing_controller;
        friend class Lateral_controller;
        team2_package::longitudinal_controller longitudinal_command;
        double steering_command;

        float prev_longitudinal_command;
        double prev_speed;
        float reverse_count;
        bool back;
        int forward_state_count;
        int forward_lock_count;
        Longitudinal_controller longitudinal_controller;
        Lateral_controller lateral_controller;

        // ADAS decision subscriber
        ros::Subscriber ACC_subscriber;
        ros::Subscriber AEB_subscriber;
        // ACC switching flag
        bool is_spacing_control;
        // AEB flag
        bool is_emergency_braking;
    public:
        Controller();
        void pose_sub_callback(const team2_package::vehicle_state::ConstPtr &pose_msg);
        void vel_ref_sub_callback(const std_msgs::Float32::ConstPtr &vel_ref_msg);
        void actual_vel_callback(const carla_msgs::CarlaSpeedometer::ConstPtr &actual_vel_msg);
        void waypoints_sub_callback(const team2_package::globalwaypoints::ConstPtr &waypoints_msg);
        void preceding_vehicle_dist_callback(const std_msgs::Float32::ConstPtr &distance_msg);

        void control_flag_sub_callback(const std_msgs::Bool::ConstPtr &msg);

        void ACC_sub_callback(const std_msgs::Bool::ConstPtr &ACC_msg);
        void AEB_sub_callback(const std_msgs::Bool::ConstPtr &AEB_msg);
        void forward_command();
        void publish_command();
        bool activate_control;
};


#endif