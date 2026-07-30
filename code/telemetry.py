import logging
from rclpy.node import Node
import json
import xml.etree.ElementTree as ET
import rclpy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
import numpy as np


class TelemetryNode(Node):
    def __init__(self, duration=2.0, cut_name='', robot_list=[]):
        super().__init__('telemetryNode')

        self.cut_name = cut_name

        self.sim_duration = int(duration * 60.0)
        self.latencies = []

        self.timer_to_delete = None
        self.current_name_for_deletion = None

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.robot_pose_listener = {}
        self.robot_camera_listener = {}
        self.robot_noisy_pose_listener = {}
        self.robot_imu_listener = {}
        self.robot_filtered_pose_listener = {}

        for name in robot_list:
            self.robot_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/pose',
                lambda msg, nm=name: self.pose_listener_callback(msg, nm),
                10
            )
            self.robot_noisy_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/noisy_pose',
                lambda msg, nm=name: self.noisy_pose_listener_callback(msg, nm),
                10
            )
            self.robot_filtered_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/filtered_pose',
                lambda msg, nm=name: self.filtered_pose_listener_callback(msg, nm),
                10
            )
            self.robot_camera_listener[name] = self.create_subscription(
                Image,
                f'/world/{cut_name}/model/{name}/link/base_link/sensor/camera_front/image',
                lambda msg, nm=name: self.camera_listener_callback(msg, nm),
                10
            )
            self.robot_imu_listener[name] = self.create_subscription(
                Imu,
                f'/world/{cut_name}/model/{name}/link/base_link/sensor/imu_sensor/imu',
                lambda msg, nm=name: self.imu_listener_callback(msg, nm),
                10
            )

        self.task_subscription = self.create_subscription(String, '/add_task', self.task_listener_callback, 10)

        self.remove_robot_listener = self.create_subscription(String, f'/remove_robot', self.remove_robot_listener_callback, 10)
        self.add_robot_listener = self.create_subscription(String, f'/add_robot', self.add_robot_listener_callback, 10)

        logging.info("Simulation start")


    def add_robot_listener_callback(self, msg):
        msg_data = msg.data.split()
        name = msg_data[0]
        # x = msg_data[1]
        # y = msg_data[2]
        # yaw = msg_data[3]

        self.robot_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/pose',
            lambda msg_g, nm=name: self.pose_listener_callback(msg_g, nm),
            10
        )
        self.robot_noisy_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/noisy_pose',
            lambda msg_g, nm=name: self.noisy_pose_listener_callback(msg_g, nm),
            10
        )
        self.robot_filtered_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/filtered_pose',
            lambda msg_g, nm=name: self.filtered_pose_listener_callback(msg_g, nm),
            10
        )
        self.robot_camera_listener[name] = self.create_subscription(
            Image,
            f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/image',
            lambda msg_g, nm=name: self.camera_listener_callback(msg_g, nm),
            10
        )
        self.robot_imu_listener[name] = self.create_subscription(
            Imu,
            f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/imu_sensor/imu',
            lambda msg_g, nm=name: self.imu_listener_callback(msg_g, nm),
            10
        )

        return


    def safe_destroy_callback(self):
        name = self.current_name_for_deletion

        self.destroy_subscription(self.robot_pose_listener.pop(name))
        self.destroy_subscription(self.robot_noisy_pose_listener.pop(name))
        self.destroy_subscription(self.robot_filtered_pose_listener.pop(name))
        self.destroy_subscription(self.robot_camera_listener.pop(name))
        self.destroy_subscription(self.robot_imu_listener.pop(name))

        self.current_name_for_deletion = None

        if self.timer_to_delete:
            self.timer_to_delete.cancel()
            self.destroy_timer(self.timer_to_delete)


    def remove_robot_listener_callback(self, msg):
        name = msg.data

        if name not in self.robot_pose_listener.keys():
            return

        if self.current_name_for_deletion == None:
            self.current_name_for_deletion = name
        else:
            return
        
        self.timer_to_delete = None

        self.timer_to_delete = self.create_timer(0.0, self.safe_destroy_callback)

        return
    

    def camera_listener_callback(self, msg, nm):
        test = 2 + 2

    def imu_listener_callback(self, msg, nm):
        now_msg = self.get_clock().now().to_msg()
        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {nm} Accel: x={msg.linear_acceleration.x:.2f}, y={msg.linear_acceleration.y:.2f}, z={msg.linear_acceleration.z:.2f}")
        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {nm} Veloc: x={msg.angular_velocity.x:.2f}, y={msg.angular_velocity.y:.2f}, z={msg.angular_velocity.z:.2f}")
        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {nm} Orien: x={msg.orientation.x:.2f}, y={msg.orientation.y:.2f}, z={msg.orientation.z:.2f}, w={msg.orientation.w:.2f}")

    def task_listener_callback(self, msg):
        now_msg = self.get_clock().now().to_msg()
        data = str(msg.data).split()

        task_x = float(data[0])
        task_y = float(data[1])

        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} New task {task_x:.2f} {task_y:.2f}")

    def pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()
        x =  msg.poses[0].position.x
        y = msg.poses[0].position.y
        q = msg.poses[0].orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_angle = np.arctan2(t3, t4)
        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {name} pose {x:.2f} {y:.2f} {theta_angle:.2f}")

    def noisy_pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()
        x =  msg.poses[0].position.x
        y = msg.poses[0].position.y
        q = msg.poses[0].orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_angle = np.arctan2(t3, t4)

        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {name} pose noise {x:.2f} {y:.2f} {theta_angle:.2f}")

    def filtered_pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()
        x =  msg.poses[0].position.x
        y = msg.poses[0].position.y
        q = msg.poses[0].orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_angle = np.arctan2(t3, t4)
        logging.info(f"{now_msg.sec}.{str(now_msg.nanosec)[:3]} Robot {name} pose filtered {x:.2f} {y:.2f} {theta_angle:.2f}")


def main(args=None):
    logging.basicConfig(
        filename='app.log', 
        level=logging.INFO, 
        format='%(message)s')
    
    with open('config.json', 'r') as file:
        data = json.load(file)

    arr = [item['name'] for item in data['robot_list']]

    with open(data["world_config"], 'r') as file:
        world_config = json.load(file)

    tree = ET.parse(world_config["world_name"])
    root = tree.getroot()
    world = root.find('world')

    cut_name = ''

    if world is not None:
        cut_name = world.get('name')

    rclpy.init(args=args)
    node = TelemetryNode(data["duration"], cut_name, arr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    logging.info("Simulation finish")


if __name__ == '__main__':
    main()