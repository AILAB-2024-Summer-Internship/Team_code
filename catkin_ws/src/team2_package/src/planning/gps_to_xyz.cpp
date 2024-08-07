#include "gps_to_xyz.hpp"

GPSXYZ::GPSXYZ() {
    lat_ref = 0.0;
    lon_ref = 0.0;
    yaw = 0.0;

    gps_sub = nh.subscribe("/carla/hero/GPS", 10, &GPSXYZ::gps_cb, this);
    imu_sub = nh.subscribe("/carla/hero/IMU", 10, &GPSXYZ::imu_cb, this);
    gps_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/carla/hero/pose", 10);
}

void GPSXYZ::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    pose = gps_to_world(msg->latitude, msg->longitude, msg->altitude);
}

void GPSXYZ::imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    double roll, pitch;
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    yaw -= M_PI / 2;
    }

void GPSXYZ::into_pose() {
    if (!yaw) {
        ROS_INFO("pose or yaw is none");
        return;
    }

    tf::Quaternion q;
    q.setRPY(0, 0, yaw);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "/map";
    pose_msg.pose.pose.position.x = pose[0];
    pose_msg.pose.pose.position.y = pose[1];
    pose_msg.pose.pose.position.z = pose[2];
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    std::vector<double> error_gps = gps_to_world(0.000005, 0.000005, 0.000005);

    for (int i = 0; i < 36; ++i) {
        pose_msg.pose.covariance[i] = 0.0;
    }
    pose_msg.pose.covariance[0] = error_gps[0];
    pose_msg.pose.covariance[7] = error_gps[0];
    pose_msg.pose.covariance[14] = error_gps[0];

    gps_pub.publish(pose_msg);
}

std::vector<double> GPSXYZ::gps_to_world(double lat, double lon, double alt) {
    std::vector<double> utm_data = gps_to_ENU(lat_ref, lon_ref, lat, lon, alt);
    std::vector<double> world(3);
    world[0] = utm_data[1];
    world[1] = utm_data[2];
    world[2] = alt;
    return world;
}

std::vector<double> GPSXYZ::gps_to_ENU(double lat_ref, double lon_ref, double lat, double lon, double z) {
    const double EARTH_RADIUS = 6378137.0;
    double scale = cos(lat_ref * M_PI / 180.0);

    double mx_ref = scale * lon_ref * M_PI * EARTH_RADIUS / 180.0;
    double my_ref = scale * EARTH_RADIUS * log(tan((90.0 + lat_ref) * M_PI / 360.0));
    double mx = scale * lon * M_PI * EARTH_RADIUS / 180.0;
    double my = scale * EARTH_RADIUS * log(tan((90.0 + lat) * M_PI / 360.0));

    std::vector<double> enu(3);
    enu[0] = 0.0;
    enu[1] = mx - mx_ref;
    enu[2] = my - my_ref;
    return enu;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_xyz");
    GPSXYZ gpsxyz;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        gpsxyz.into_pose();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
