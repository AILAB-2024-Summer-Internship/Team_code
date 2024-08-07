#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Header.h>
#include "team2_package/keyboard_msg.h"

class Pseudo_controller {
    private:
        ros::NodeHandle nh;
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        ros::Publisher control_cmd_publisher;
        ros::Subscriber keyboard_subscriber;
        bool back = false;
        float throttle = 0;
        float steer = 0;
        bool brake = false;
    public:
        Pseudo_controller();
        void keyboard_msg_callback(const team2_package::keyboard_msg& keyboard_msg);
        void publish();
};

Pseudo_controller::Pseudo_controller(){
    control_cmd_publisher = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd",1);
    keyboard_subscriber = nh.subscribe("keyboard_topic", 100, &Pseudo_controller::keyboard_msg_callback, this);
}

void Pseudo_controller::keyboard_msg_callback(const team2_package::keyboard_msg& keyboard_msg) {
    throttle = keyboard_msg.fb_speed * 0.7;
    steer = keyboard_msg.yaw * 0.7;
    if(keyboard_msg.gear == 1) {
        back = false;
    } else {
        back = true;
    }
}

void Pseudo_controller::publish(){

    control_cmd.header.stamp = ros::Time::now();
    control_cmd.header.frame_id = "hero";
    control_cmd.throttle = throttle;
    control_cmd.steer = steer;
    control_cmd.brake = int(brake);
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