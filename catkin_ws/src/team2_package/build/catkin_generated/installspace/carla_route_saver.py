#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from carla_msgs.msg import CarlaRoute
import os

def callback(data):
    # Extract the data from the message
    road_opts_str = ' '.join(str(x) for x in data.road_options)
    poses_str = ' '.join(f"({pose.position.x}, {pose.position.y}, {pose.position.z})" for pose in data.poses)
    
    # Generate output string
    output_data = f"Header:\n{data.header}\nRoad Options:\n{road_opts_str}\nPoses:\n{poses_str}\n"

    # Save to file on the desktop
    desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
    file_path = os.path.join(desktop_path, 'global_plan.txt')
    with open(file_path, 'w') as file:
        file.write(output_data)
    rospy.loginfo("Global plan saved to {}".format(file_path))

def listener():
    rospy.init_node('carla_route_saver')
    rospy.Subscriber("/carla/hero/global_plan", CarlaRoute, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

#listener()