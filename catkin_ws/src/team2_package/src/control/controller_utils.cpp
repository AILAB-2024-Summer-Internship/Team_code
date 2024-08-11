#include "controller.hpp"
#include <thread>
#include <chrono>
// linearMap 함수 정의
double linearMap(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}

// Initialize static variables
float Longitudinal_controller::ACC::speed_controller::i_term = 0;
float Longitudinal_controller::ACC::speed_controller::last_error = 0;
float Longitudinal_controller::ACC::speed_controller::command = 0;
int Longitudinal_controller::ACC::speed_controller::reverse_check_count = 0;
float Longitudinal_controller::ACC::spacing_controller::last_error = 0;
float Longitudinal_controller::ACC::spacing_controller::command = 0;

// Speed controller
void Longitudinal_controller::ACC::speed_controller::operator()(Controller& controller) {
    float vel_ref = (20.0 - 18.5*(controller.steering_command))/3.6;
    
    //float vel_ref = 20.0 * KM_PER_H_TO_M_PER_S;
    std::cout << "p gain: " << p_gain << " , i gain: " << i_gain << std::endl;
    std::cout << "i team: " << i_term << std::endl;
    std::cout << "reference velocity: " << vel_ref << std::endl;
    
    //// j f p e /////
    float current_vel = controller.actual_vel; 
    error = vel_ref - controller.actual_vel;
    //////////////////

    std::cout << "actual vel: " << controller.actual_vel  << " / " << "vel error: " << error << std::endl;

    i_term += i_gain * error * time_interval;
    command = p_gain * error + i_term + d_gain * (error - last_error) / time_interval;
    ROS_INFO("longitudinal command: %.5f", command);
    
    // lower controller
    // command = linearMap(command, static_cast<float>(-MAX_ACCELERATION), static_cast<float>(MAX_ACCELERATION) , -1.0, 1.0);
    
    last_error = error;

    command = (command > 1.0 ? 1.0 : (command < -1.0 ? -1.0 : command));
    //std::cout << "Final longitudinal command: " << command << std::endl;

    // Check for sudden speed drop or lack of acceleration
    float speed_change = fabs(controller.prev_speed - controller.actual_vel);
    float accel_threshold = 0.125; // Define a threshold for minimum expected acceleration

    // Chanwoo 파일에 올린 임시 코드로 테스트해보기
    // for a moment ...
    // Add a counter to ensure the condition persists for a certain time
    // if (speed_change >= 7.5 || (command > 0.95 && speed_change < accel_threshold)) {
    //     reverse_check_count++;

    //     if (reverse_check_count >= 20) { // Maintain condition for 20 iterations
    //         std::cout << "------------change gear to reverse--------------" << std::endl;
    //         controller.back = true;
    //         reverse_check_count = 0;
    //     }
    // } else { reverse_check_count = 0; } // Reset if condition is not met

    if (command >= 0) {
        controller.longitudinal_command.throttle = command;
        controller.longitudinal_command.brake = 0;
    } else {
        controller.longitudinal_command.throttle = 0;
        controller.longitudinal_command.brake = -command;
    }

    /////// Just for performance evaluation ///////
    controller.longitudinal_evaluation(vel_ref, current_vel);
    ///////////////////////////////////////////////
}

// Spacing controller
void Longitudinal_controller::ACC::spacing_controller::operator()(Controller& controller) {
    error = desired_distance - controller.preceding_vehicle_dist + preceding_vehicle_length;
    command = -p_gain * error - d_gain * (error - last_error) / time_interval;
    last_error = error;

    if (command >= 0) {
        controller.longitudinal_command.throttle = command;
        controller.longitudinal_command.brake = 0;
    } else {
        controller.longitudinal_command.throttle = 0;
        controller.longitudinal_command.brake = -command;
    }
}

void Longitudinal_controller::AEB_ACC_control(Controller& controller){
    // AEB
    if(controller.is_emergency_braking){
        controller.longitudinal_command.throttle = 0;
        controller.longitudinal_command.brake = 1.0;
    }
    // ACC
    else {
        // spacing controller
        if (controller.is_spacing_control) {
            ACC::spacing_controller ACC_controller;
            ACC_controller(controller);
        }
        // speed controller
        else {
            ACC::speed_controller ACC_controller;
            ACC_controller(controller);
        }
    }
}

void Lateral_controller::polyfit_waypoints(const Controller& controller) {

    least_squares_x = cv::Mat::zeros(controller.waypoints.x.size(),4,CV_32F);
    least_squares_y = cv::Mat::zeros(controller.waypoints.x.size(),1,CV_32F);
    if (controller.waypoints.x.size() < 4) {

        //ROS_ERROR("Not enough waypoints to perform polynomial fitting.");
        return;
    }
    for(int index = 0; index < controller.waypoints.x.size(); index++){
        least_squares_y.at<float>(index,0) = controller.waypoints.y[index];
        least_squares_x.at<float>(index,0) = pow(controller.waypoints.x[index],3);
        least_squares_x.at<float>(index,1) = pow(controller.waypoints.x[index],2);
        least_squares_x.at<float>(index,2) = controller.waypoints.x[index];
        least_squares_x.at<float>(index,3) = 1.0;
    }
    // least squares
    if (cv::solve(least_squares_x, least_squares_y, coefficients, cv::DECOMP_SVD)) {
        //ROS_INFO("Polynomial fitting successful.");
    } else {
        //ROS_ERROR("Polynomial fitting failed. The system might be under-determined.");
    }
}

void Lateral_controller::calculate_curvature(const Controller& controller){
    polyfit_waypoints(controller);
    std::vector<float> curvature_list;
    for(int i = 0; i < controller.waypoints.x.size(); i++) {
        float first_derivative = 3*coefficients.at<float>(0,0)*least_squares_x.at<float>(i,1) + 2*coefficients.at<float>(1,0)*least_squares_x.at<float>(i,2);
        float second_derivative = 6*coefficients.at<float>(0,0)*least_squares_x.at<float>(i,2) + 2*coefficients.at<float>(1,0);
        curvature_list.push_back(fabs(second_derivative)/pow((1+pow(first_derivative,2)),1.5));
    }
    curvature_list.clear();
    //curvature = std::accumulate(curvature_list.begin(), curvature_list.end(), 0.0) / controller.waypoints.x.size(); // mean of curvature
    auto max_iter = std::max_element(curvature_list.begin(), curvature_list.end());

    if (max_iter != curvature_list.end()) {
        curvature = *max_iter;
        std::cout << "curvature: " << curvature << std::endl;
    } else {
        std::cout << "curvature list is empty." << std::endl;
    }
}

void Lateral_controller::find_look_ahead_distance(const Controller& controller){
    calculate_curvature(controller);
    float curvature_term = 1/sqrt(ld_gain_a + ld_gain_b*curvature); // 현재 종방향 속도에 대한 term도 고려해야 할 듯
    float velocity_term = ld_gain_vel*controller.actual_vel;
    look_ahead_distance = weighted_term*curvature_term + (1-weighted_term)*velocity_term;
    std::cout << "real lfd: " << look_ahead_distance << std::endl;
}

double Lateral_controller::calculateDistance(const geometry_msgs::Point& rear_wheel_position, const geometry_msgs::Point& point) {
    double distance = sqrt(pow(rear_wheel_position.x - point.x, 2) + pow(rear_wheel_position.y - point.y, 2));
    return fabs(distance - look_ahead_distance);
}

void Lateral_controller::pure_pursuit(Controller& controller) {
    //std::cout << "Entering pure_pursuit" << std::endl;

    // Initialize the closest_point_finder vector
    controller.closest_point_finder.clear();
    find_look_ahead_distance(controller);

    rear_wheel_position.x = controller.ego_vehicle_pose.x - (WHEELBASE / 2) * std::cos(controller.ego_vehicle_pose.yaw);
    rear_wheel_position.y = controller.ego_vehicle_pose.y - (WHEELBASE / 2) * std::sin(controller.ego_vehicle_pose.yaw);

    rear_wheel_position.yaw = controller.ego_vehicle_pose.yaw;

    std::cout << "Rear wheel position: x = " << rear_wheel_position.x << ", y = " << rear_wheel_position.y << ", yaw = " << rear_wheel_position.yaw << std::endl;

    // Check the size of waypoints
    if (controller.waypoints.x.size() < 10 || controller.waypoints.y.size() < 10) {
        //std::cerr << "Error: Not enough waypoints to perform polynomial fitting." << std::endl;
        return;
    }

    for (size_t i = 0; i < controller.waypoints.x.size(); ++i) {
        geometry_msgs::Point point;
        point.x = controller.waypoints.x[i];
        point.y = controller.waypoints.y[i];
        controller.closest_point_finder.push_back(point);
    }

    // Check if closest_point_finder is empty
    if (controller.closest_point_finder.empty()) {
        //std::cerr << "Error: closest_point_finder is empty." << std::endl;
        return;
    }

    geometry_msgs::Point rear_wheel_point;
    rear_wheel_point.x = rear_wheel_position.x;
    rear_wheel_point.y = rear_wheel_position.y;

    auto target_point_it = std::min_element(controller.closest_point_finder.begin(), controller.closest_point_finder.end(),
                                          [this, &rear_wheel_point](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
                                              double dist1 = this->calculateDistance(rear_wheel_point, p1);
                                              double dist2 = this->calculateDistance(rear_wheel_point, p2);
                                              return dist1 < dist2;
                                          });

    if (target_point_it == controller.closest_point_finder.end()) {
        //std::cerr << "Error: No valid target point found." << std::endl;
        return;
    }

    geometry_msgs::Point target_point = *target_point_it;

    std::cout << "Target Point: x = " << target_point.x << ", y = " << target_point.y << std::endl;

    alpha = atan2((target_point.y - rear_wheel_position.y), (target_point.x - rear_wheel_position.x)) - rear_wheel_position.yaw;
    float dist = sqrt(pow((target_point.y - rear_wheel_position.y),2)+pow((target_point.x - rear_wheel_position.x),2));
    //std::cout << "look ahead distance: " << dist << std::endl;

    if (alpha > M_PI) {
        alpha -= 2 * M_PI;
    } else if (alpha < -M_PI) {
        alpha += 2 * M_PI;
    }

    //std::cout << "current yaw: " << rear_wheel_position.yaw << std::endl;
    //std::cout << "target angle: " << atan2((target_point.y - rear_wheel_position.y), (target_point.x - rear_wheel_position.x)) << std::endl;
    //std::cout << "alpha: " << alpha*RAD_TO_DEG << std::endl;
    double raw_steering_angle = -atan2(2 * WHEELBASE * sin(alpha), dist);
    //std::cout << "raw steering angle: " << raw_steering_angle << std::endl;

    double cliped_steering_angle = (MAX_STEER_ANGLE < raw_steering_angle ? MAX_STEER_ANGLE : (-MAX_STEER_ANGLE > raw_steering_angle ? -MAX_STEER_ANGLE : raw_steering_angle)); // -1 ~ 1 rad

    //cliped_steering_angle = linearMap(cliped_steering_angle, static_cast<double>(-MAX_STEER_ANGLE), static_cast<double>(MAX_STEER_ANGLE), -1.0, 1.0);

    double target_steering_angle = (1.0 < cliped_steering_angle ? 1.0 : (-1.0 > cliped_steering_angle ? -1.0 : cliped_steering_angle));

    if(fabs(cliped_steering_angle) < STEERING_THRESHOLD){
        controller.steering_command = 0.0;
    }

    else {
        if(controller.back){ controller.steering_command = -target_steering_angle; }
        else { controller.steering_command = target_steering_angle; }
    }

    //////////////////////////// j f p e ///////////////////////////////
    float dx = target_point.x - controller.ego_vehicle_pose.x;
    float dy = target_point.y - controller.ego_vehicle_pose.y;
    float heading_angle_error = atan2(dy, dx) - rear_wheel_position.yaw;
    float XTE = sqrt(pow(dx,2) + pow(dy,2)) * sin(heading_angle_error);
    controller.lateral_evaluation(XTE, heading_angle_error);
    ////////////////////////////////////////////////////////////////////


    std::cout << "lateral command: " << controller.steering_command << std::endl;
}

void Controller::pose_sub_callback(const team2_package::vehicle_state::ConstPtr &pose_msg) { ego_vehicle_pose = *pose_msg; }
void Controller::vel_ref_sub_callback(const std_msgs::Float32::ConstPtr &vel_ref_msg) { vel_ref = vel_ref_msg->data; }
void Controller::actual_vel_callback(const carla_msgs::CarlaSpeedometer::ConstPtr &actual_vel_msg) { actual_vel = actual_vel_msg->speed; }
void Controller::preceding_vehicle_dist_callback(const std_msgs::Float32::ConstPtr &distance_msg){ preceding_vehicle_dist = distance_msg->data; }
void Controller::control_flag_sub_callback(const std_msgs::Bool::ConstPtr &msg) { activate_control = msg->data; }
void Controller::waypoints_sub_callback(const team2_package::globalwaypoints::ConstPtr &waypoints_msg) {
    waypoints.road_options.resize(10); // 10: number_of_waypoint
    waypoints.x.resize(10);
    waypoints.y.resize(10);
    waypoints.road_options.clear();
    waypoints.x.clear();
    waypoints.y.clear();
    waypoints = *waypoints_msg;
}

void Controller::ACC_sub_callback(const std_msgs::Bool::ConstPtr &ACC_msg){ is_spacing_control = ACC_msg->data; }
void Controller::AEB_sub_callback(const std_msgs::Bool::ConstPtr &AEB_msg){ is_emergency_braking = AEB_msg->data; }

Controller::Controller()
: actual_vel(0), forward_state_count(0), forward_lock_count(0), lateral_controller(), back(false),
is_emergency_braking(false), is_spacing_control(false), reverse_count(0), activate_control(false) {
    ego_vehicle_pose_subscriber = nh_.subscribe("/carla/hero/localization", 10, &Controller::pose_sub_callback, this);
    // These topic names are temporary. They will be named appropriately by planner (Ja hyun)
    //vel_ref_subscriber = nh_.subscribe("/carla/hero/global_stop", 10, &Controller::vel_ref_sub_callback, this); who did it ;;
    actual_vel_subscriber = nh_.subscribe("/carla/hero/Speed", 10, &Controller::actual_vel_callback, this);
    waypoints_subscriber = nh_.subscribe("/carla/hero/my_global_plan", 10, &Controller::waypoints_sub_callback, this);
    preceding_vehicle_dist_subscriber = nh_.subscribe("/carla/hero/preceding_vehicle_distance", 1, &Controller::preceding_vehicle_dist_callback, this);
    control_flag_subscriber = nh_.subscribe("/carla/control_flag", 1, &Controller::control_flag_sub_callback, this);

    AEB_subscriber = nh_.subscribe("/carla/hero/global_stop", 1, &Controller::AEB_sub_callback, this);
    ACC_subscriber = nh_.subscribe("/carla/hero/global_ACC", 1, &Controller::ACC_sub_callback, this);

    control_cmd_publisher = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd",1);

    //////////////////////////////// j f p e /////////////////////////////////////////
    ref_speed_publisher = nh_.advertise<std_msgs::Float32>("/carla/hero/ref_speed", 1);
    actual_speed_publisher = nh_.advertise<std_msgs::Float32>("/carla/hero/actual_speed", 1);
    XTE_publisher = nh_.advertise<std_msgs::Float32>("/carla/hero/XTE", 1);
    yaw_error_publisher = nh_.advertise<std_msgs::Float32>("/carla/hero/heading_angle_error", 1);
    //////////////////////////////////////////////////////////////////////////////////

    control_cmd.header.frame_id="/hero";
    control_cmd.hand_brake = false;
    control_cmd.manual_gear_shift = false; // auto gear
}

void Controller::forward_command() {
    control_cmd.reverse = false;
    // Lateral control command
    lateral_controller.pure_pursuit(*this);
    control_cmd.steer = steering_command;

    std::cout << "---------------lateral control end ----------------" << std::endl;
    // Longitudinal control command
    longitudinal_controller.AEB_ACC_control(*this);
    control_cmd.throttle = longitudinal_command.throttle;
    control_cmd.brake = longitudinal_command.brake;
    std::cout << "---------------longitudinal control end ----------------" << std::endl;
}

void Controller::publish_command() {
    control_cmd.header.stamp = ros::Time::now();

    // Count how long the vehicle has been moving forward
    forward_state_count++;

    // Lock the vehicle in forward mode for a certain number of cycles
    if (forward_lock_count > 0) {
        forward_lock_count--;
        forward_command();

    } else if (back) {
        // Only allow reverse if forward cycles have been maintained
        if (forward_state_count >= 150) { // Ensure some forward cycles
            if (reverse_count == 0) {
                forward_state_count = 0; // Reset forward count for next reverse cycle
            }
            std::cout << "--------------------reverse gear----------------------" << std::endl;
            control_cmd.reverse = true;
            control_cmd.throttle = 0.5; // Set throttle for reversing
            control_cmd.brake = 0.0;
            control_cmd.steer = 0.0;
            reverse_count++;
            lateral_controller.pure_pursuit(*this);
            //longitudinal_controller.AEB_ACC_control(*this);
            //control_cmd.throttle = longitudinal_command.throttle;
            control_cmd.throttle = 0.5;
            control_cmd.steer = steering_command;
            // After reversing for 30 cycles, switch to forward mode
            if (reverse_count >= 100) {
                back = false;  // Set back to false to enable forward mode
                forward_lock_count = 150; // Set lock count to prevent immediate reverse
                std::cout << "---------gear change: reverse to forward----------" << std::endl;
                reverse_count = 0;
            }
        } else {
            forward_command(); // Continue in forward mode if not enough forward cycles
        }
    } else {
        forward_command();
    }

    control_cmd_publisher.publish(control_cmd);
    //std::cout << "---------------wait for new control ----------------" << std::endl;

    prev_speed = static_cast<double>(actual_vel);
    prev_longitudinal_command = control_cmd.throttle;
}


//////////////////////////////// j f p e ///////////////////////////////
void Controller::longitudinal_evaluation(float ref_vel, float act_vel) {
    std_msgs::Float32 ref;
    ref.data = ref_vel;
    std_msgs::Float32 act;
    act.data = act_vel;

    ref_speed_publisher.publish(ref);
    actual_speed_publisher.publish(act);
}

void Controller::lateral_evaluation(float xte, float yaw_error) {
    std_msgs::Float32 XTE;
    std_msgs::Float32 Yaw_error;
    XTE.data = xte;
    Yaw_error.data = yaw_error;
    XTE_publisher.publish(XTE);
    yaw_error_publisher.publish(Yaw_error);
}
////////////////////////////////////////////////////////////////////////