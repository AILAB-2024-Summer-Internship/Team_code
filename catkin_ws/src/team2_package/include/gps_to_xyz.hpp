#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>

class GPSXYZ {
    public:
        std::vector<double> pose;
        double yaw;

        GPSXYZ();

        void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
        void into_pose();

        std::vector<double> gps_to_world(double lat, double lon, double alt);
        std::vector<double> gps_to_ENU(double lat_ref, double lon_ref, double lat, double lon, double z);

    private:
        ros::NodeHandle nh;
        ros::Subscriber gps_sub;
        ros::Subscriber imu_sub;
        ros::Publisher gps_pub;
        
        double lat_ref;
        double lon_ref;
};
