#include "localizer.hpp"

ros::Duration time_interval(0.02); // 초기화 시에 ros::Duration 사용

std::array<double,3> Wgs84toEnu::gps_to_UTM(double lat, double lon, double alt) {

    double scale = cos(ref_latitude * M_PI / 180.0);

    double mx_ref = ref_longitude * DEG_TO_RAD * EARTH_RADIUS;
    double my_ref = EARTH_RADIUS * log(tan((90.0 + ref_latitude) * M_PI / 360.0));
    double mx = lon * DEG_TO_RAD * EARTH_RADIUS;
    double my = EARTH_RADIUS * log(tan((90.0 + lat) * M_PI / 360.0));

    std::array<double,3> ENU;
    ENU[0] = mx - mx_ref;
    ENU[1] = my - my_ref;
    return ENU;
}

double angle_clip(double angle){
    return (angle > M_PI ? angle_clip(angle-2*M_PI) : (angle < -M_PI ? angle_clip(angle+2*M_PI) : angle));
}

void Wgs84toEnu::callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gps_position = gps_to_UTM(msg->latitude, msg->longitude, msg->altitude);
}

void Imu_processor::callback(const sensor_msgs::Imu::ConstPtr &inertial_meas) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(inertial_meas->orientation,q);
    tf::Matrix3x3(q).getRPY(orientation.x, orientation.y, orientation.z);
    angular_velocity.x = inertial_meas->angular_velocity.x;
    angular_velocity.y = inertial_meas->angular_velocity.y;
    angular_velocity.z = inertial_meas->angular_velocity.z;
    linear_accel.x = inertial_meas->linear_acceleration.x;
    linear_accel.y = inertial_meas->linear_acceleration.y;
    linear_accel.z = inertial_meas->linear_acceleration.z;
}

void Localizer::speed_sub_callback(const std_msgs::Float32::ConstPtr &speed_msg){ current_speed = speed_msg->data; }

Localizer::Localizer() {
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/carla/hero/IMU", 1, boost::bind(&Imu_processor::callback, &imu_processor, _1)); //template specification for boost::bind
    gps_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("/carla/hero/GPS", 1, boost::bind(&Wgs84toEnu::callback, &wgs84_to_enu, _1));
    speed_subscriber = nh.subscribe("/carla/hero/speed", 1, &Localizer::speed_sub_callback, this);
    localization_publisher = nh.advertise<team2_package::vehicle_state>("/carla/hero/localization", 1);
    EKF = cv::KalmanFilter(3,2,0,CV_64F);
    EKF.transitionMatrix = cv::Mat::eye(3,3,CV_64F); // F
    EKF.measurementMatrix = cv::Mat::eye(3,3,CV_64F); // H
    EKF.errorCovPre = cv::Mat::zeros(3,3,CV_64F); // priori P
    EKF.errorCovPost = cv::Mat::zeros(3,3,CV_64F); // posterirori P

    EKF.processNoiseCov = cv::Mat::eye(3,3,CV_64F)*0.8; // Q
    EKF.processNoiseCov.at<double>(2,2) = 0.5*DEG_TO_RAD;

    EKF.measurementNoiseCov = cv::Mat::eye(3,3, CV_64F)*0.55; // R
    EKF.measurementNoiseCov.at<double>(2,2) = 0.5*DEG_TO_RAD;

    EKF.statePost.at<double>(0,0) = 6770.8;
    EKF.statePost.at<double>(1,0) = -5443.4;
    EKF.statePost.at<double>(2,0) = 89.5*DEG_TO_RAD;
    measurement = cv::Mat(3,1,CV_64F);

    last_time = ros::Time::now();
}

cv::Mat Localizer::f() {
    cv::Mat newState(3,1,CV_64F);
    double yaw_rate = imu_processor.angular_velocity.z;

        newState.at<double>(0,0) = EKF.statePost.at<double>(0,0) + current_speed * time_interval.toSec() * cos(EKF.statePost.at<double>(2,0));
        newState.at<double>(1,0) = EKF.statePost.at<double>(1,0) + current_speed * time_interval.toSec() * sin(EKF.statePost.at<double>(2,0));
        newState.at<double>(2,0) = EKF.statePost.at<double>(2,0) + yaw_rate * time_interval.toSec();

    return newState;
}

void Localizer::dead_reckoning(){
    // Extended Kalman Filter
    double yaw = EKF.statePost.at<double>(2, 0);
    yaw = angle_clip(yaw); // -PI ~ PI

    time_interval = ros::Time::now() - last_time;
    last_time = ros::Time::now();

    // F matrix update
    EKF.transitionMatrix.at<double>(0,2) = -current_speed * time_interval.toSec() * sin(yaw);
    EKF.transitionMatrix.at<double>(1,2) = current_speed * time_interval.toSec() * cos(yaw);

    //predict (update priori P)
    EKF.predict();
    
    // prediction by nonlinear function
    EKF.statePre = f();
    
    //measurement
    measurement.at<double>(0,0) = wgs84_to_enu.gps_position[0];
    measurement.at<double>(1,0) = wgs84_to_enu.gps_position[1];
    measurement.at<double>(2,0) = imu_processor.orientation.z;

    // correct
    EKF.correct(measurement);
    // EKF.errorCovPost = (cv::Mat::eye(3, 3, CV_64F) - EKF.gain * EKF.measurementMatrix) * 
    //                EKF.errorCovPre * 
    //                (cv::Mat::eye(3, 3, CV_64F) - EKF.gain * EKF.measurementMatrix).t() + 
    //                EKF.gain * EKF.measurementNoiseCov * EKF.gain.t();
}

void Localizer::local_publish() {
    dead_reckoning();
    vehicle_localization.x = EKF.statePost.at<double>(0,0);
    vehicle_localization.y = EKF.statePost.at<double>(1,0);
    vehicle_localization.yaw = EKF.statePost.at<double>(2,0);
    localization_publisher.publish(vehicle_localization);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        localizer.local_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}