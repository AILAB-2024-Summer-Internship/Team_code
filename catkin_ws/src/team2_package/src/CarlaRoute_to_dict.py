import os
import argparse
import rosbag
import csv
from carla_msgs.msg import CarlaRoute

def main():
    """Extract data from a rosbag and save as a CSV file."""
    parser = argparse.ArgumentParser(description="Extract CarlaRoute messages from a ROS bag and save as a CSV file.")
    parser.add_argument("bag_file", help="Input ROS bag file.")
    parser.add_argument("output_file", help="Output CSV file.")
    parser.add_argument("topic", help="Topic name for CarlaRoute messages.")

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag_file, "r")
    fieldnames = ['seq', 'stamp_secs', 'stamp_nsecs', 'frame_id', 'road_option', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']

    with open(args.output_file, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            if isinstance(msg, CarlaRoute):
                for i, pose in enumerate(msg.poses):
                    writer.writerow({
                        'seq': msg.header.seq,
                        'stamp_secs': msg.header.stamp.secs,
                        'stamp_nsecs': msg.header.stamp.nsecs,
                        'frame_id': msg.header.frame_id,
                        'road_option': msg.road_options[i],
                        'position_x': pose.position.x,
                        'position_y': pose.position.y,
                        'position_z': pose.position.z,
                        'orientation_x': pose.orientation.x,
                        'orientation_y': pose.orientation.y,
                        'orientation_z': pose.orientation.z,
                        'orientation_w': pose.orientation.w
                    })
                    print(f"Written message with seq: {msg.header.seq}, road option: {msg.road_options[i]}")

    bag.close()

if __name__ == '__main__':
    main()

