#include <ros/ros.h>
#include "team2_package/keyboard_msg.h"
#include "keyboard.hpp"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <vector>


// global value
char key(' ');
float fb_speed = 0;
float yaw = 0;
int gear = 1;
bool brake;

// keyboard map
std::map<char,std::vector<float>> keyboard_map {
    {'w',{1,0}}, // front speed (front)
    {'s',{0,0}}, // backward speed (back)
    {'a',{0,-1}}, // yaw (ccw)
    {'d',{0,1}}, // yaw (cw)
    {'q',{0,0}}
};

// non-blocking keyboard input on terminal
char getch(){
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
};

KeyboardNode::KeyboardNode() {
    pub = nh.advertise<team2_package::keyboard_msg>("keyboard_topic", 1000);
}

KeyboardNode::~KeyboardNode() {}

void KeyboardNode::function() {
    brake = false;
    key = getch();

    if (keyboard_map.count(key) == 1) {
        if (key == 'q') {
            gear *= -1;
        } else if (key == 's') {
            msg.brake = true;
        }
        fb_speed = keyboard_map[key][0];
        yaw = keyboard_map[key][1];
    }

    msg.fb_speed = fb_speed;
    msg.yaw = yaw;
    msg.gear = gear;
    msg.brake = brake;
    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_node");
    KeyboardNode keyboard_node;
    while(ros::ok()){
        keyboard_node.function();
        ros::spinOnce();
    }
    return 0;
}