#!/usr/bin/env python
from leaderboard.autoagents.ros1_agent import ROS1Agent
from leaderboard.autoagents.autonomous_agent import Track
import carla
import gc

def get_entry_point():
    return 'TEAM2_AGENT'

class TEAM2_AGENT(ROS1Agent):
    def setup(self, path_to_conf_file):
        self.track = Track.MAP

    def get_ros_entrypoint(self):
        entrypoint = {
            "package": "team2_package",
            "launch_file": "test.launch"
        }
        print("ROS entrypoint: {}".format(entrypoint))
        return entrypoint

    def sensors(self):

        sensors = [
            {'type': 'sensor.camera.rgb', 'id': 'rgb_view',
             'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 800, 'height': 600, 'fov': 90},
            {'type': 'sensor.camera.rgb', 'id': 'Camera_for_traffic_light',
              'x': 2.5, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 1200, 'height': 900, 'fov': 90},
            {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0},
            {'type': 'sensor.other.gnss', 'id': 'gnss',
             'x': 0.7, 'y': -0.4, 'z': 1.60},
            {'type': 'sensor.other.imu', 'id': 'IMU',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0},
            {'type': 'sensor.speedometer', 'id': 'Speed'},
            {'type': 'sensor.opendrive_map', 'id': 'OpenDRIVE', 'reading_frequency': 1e-6},

            ################################ 여기서 부턴 manual_control.py를 위한 강제센서들 #################################
            {'type' : 'sensor.pseudo.odom', 'id' : 'odometry'},
            {'type' : 'sensor.other.lane_invasion', 'id' : 'lane_invasion', "x": 0.0, "y": 0.0, "z": 0.0},
            {'type' : 'sensor.other.collision', 'id' : 'collision', "x": 0.0, "y": 0.0, "z": 0.0},
            {'type' : 'sensor.other.obstacle', 'id' : 'obstacle'}

        ]
        return sensors
    
    def destroy(self):
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)

            world = client.get_world()

            # 모든 액터를 가져와 삭제합니다
            actors = world.get_actors()
            for actor in actors:
                if actor.is_alive:
                    actor.destroy()
                    
            print("All actors destroyed successfully.")

        except Exception as e:
            print(f"Error during destruction: {e}")

        finally:
            # 가비지 컬렉션을 강제로 수행합니다
            gc.collect()
            print("Garbage collection performed.")
