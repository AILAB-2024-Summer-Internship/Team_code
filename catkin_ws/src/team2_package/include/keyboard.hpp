#ifndef __KEYBOARD__HPP__
#define __KEYBOARD__HPP__

#include <ros/ros.h>
#include "team2_package/keyboard_msg.h"

class KeyboardNode {
public:
    KeyboardNode();
    ~KeyboardNode();

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    team2_package::keyboard_msg msg;

public:
    void function();
};


#endif