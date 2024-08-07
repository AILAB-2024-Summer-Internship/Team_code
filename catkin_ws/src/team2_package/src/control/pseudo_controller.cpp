#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Header.h>

class Pseudo_controller {
    private:
        ros::NodeHandle nh_;
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        ros::Publisher control_cmd_publisher;
        bool back = false;
    public:
        Pseudo_controller();
        void publish();
};

Pseudo_controller::Pseudo_controller(){
    control_cmd_publisher = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd",1);
}

void Pseudo_controller::publish(){
    control_cmd.header.stamp = ros::Time::now();
    control_cmd.header.frame_id = "/hero";
    control_cmd.throttle = 0.0;
    control_cmd.steer = 0.0;
    control_cmd.brake = 0.0;
    control_cmd.hand_brake = false;
    control_cmd.manual_gear_shift = false;

    if(back){
        control_cmd.reverse = true;
        control_cmd.gear = -1;
    }
    else {
        control_cmd.reverse = false;
        control_cmd.gear = 1;
    }

    control_cmd_publisher.publish(control_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pseudo_controller");
    Pseudo_controller pseudo_controller;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        pseudo_controller.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}