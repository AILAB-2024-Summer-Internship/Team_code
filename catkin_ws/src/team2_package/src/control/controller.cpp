#include "controller_utils.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    Controller controller;
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        //if(controller.activate_control){
            controller.publish_command();
            ros::spinOnce();
            loop_rate.sleep();
        //}
    }
    return 0;
}