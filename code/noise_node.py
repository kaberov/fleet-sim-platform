#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import numpy as np
import json
import xml.etree.ElementTree as ET


class NoiseNode(Node):
    def __init__(self, seed=42, noise=0.01, robot_list=[]):
        super().__init__('noiseNode')

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.timer_to_delete = None
        self.current_name_for_deletion = None

        self.rng = np.random.default_rng(seed=seed)
        self.noise = noise

        self.robot_list = robot_list

        self.robot_pose_listener = {}
        self.robot_noise_pose_publisher = {}

        for name in robot_list:
            self.robot_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/pose',
                lambda msg, nm=name: self.pose_listener_callback(msg, nm),
                10
            )
            self.robot_noise_pose_publisher[name] = self.create_publisher(
                PoseArray, 
                f'/model/{name}/noisy_pose', 
                10
            )

        self.remove_robot_listener = self.create_subscription(String, f'/remove_robot', self.remove_robot_listener_callback, 10)
        self.add_robot_listener = self.create_subscription(String, f'/add_robot', self.add_robot_listener_callback, 10)


    def add_robot_listener_callback(self, msg):
        msg_data = msg.data.split()
        name = msg_data[0]
        # x = msg_data[1]
        # y = msg_data[2]
        # yaw = msg_data[3]

        self.robot_noise_pose_publisher[name] = self.create_publisher(PoseArray, f'/model/{name}/noisy_pose', 10)
        self.robot_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/pose',
            lambda msg_g, nm=name: self.pose_listener_callback(msg_g, nm),
            10
        )

        return


    def safe_destroy_callback(self):
        name = self.current_name_for_deletion

        self.destroy_publisher(self.robot_noise_pose_publisher.pop(name))
        self.destroy_subscription(self.robot_pose_listener.pop(name))

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
    
    
    def pose_listener_callback(self, msg, nm):
        msg.poses[0].position.x = msg.poses[0].position.x + self.rng.uniform(-self.noise, self.noise)
        msg.poses[0].position.y = msg.poses[0].position.y + self.rng.uniform(-self.noise, self.noise)

        self.robot_noise_pose_publisher[nm].publish(msg)


def main(args=None):
    with open('config.json', 'r') as file:
        data = json.load(file)

    robot_list = [item['name'] for item in data['robot_list']]

    rclpy.init(args=args)
    node = NoiseNode(seed=data['seed'], noise=data['noise'], robot_list=robot_list)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()