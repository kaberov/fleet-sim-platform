#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from geometry_msgs.msg import PoseArray
from collections import deque
import numpy as np
import json
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Pose, Point, Quaternion


class EventGenerator(Node):
    def __init__(self, duration=2.0, cut_name='', event_list=[], model_dict={}, robot_list={}):
        super().__init__('eventGeneratorNode')

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.timer_to_delete = None
        self.current_name_for_deletion = None

        self.cylinder_id = 0
        self.sim_duration = int(duration * 60.0)
        self.start_sim_time = None
        self.event_list = deque(event_list)
        self.model_dict = model_dict
        self.active_triggers = []
        self.robot_list = robot_list
        self.dynamic_obstacles = []

        self.robot_pose_listener = {}
        self.robot_batery_publisher = {}
        self.robot_batery = {}

        self.task_publisher = self.create_publisher(String, '/add_task', 10)
        self.clock_subscription = self.create_subscription(Clock, '/clock', self.clock_listener_callback, 10)

        self.client_set_pose = self.create_client(SetEntityPose, f'/world/{cut_name}/set_pose')
        self.client_spawn = self.create_client(SpawnEntity, f'/world/{cut_name}/create')

        for name in robot_list.keys():
            self.robot_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/pose',
                lambda msg, nm=name: self.pose_listener_callback(msg, nm),
                10
            )
            self.robot_batery[name]=1.0
            self.robot_batery_publisher[name] = self.create_publisher(
                Float64, 
                f'/model/{name}/batery', 
                10
            )

        self.publisher_control = self.create_publisher(String, '/control', 10)

        self.remove_robot_listener = self.create_subscription(String, f'/remove_robot', self.remove_robot_listener_callback, 10)
        self.add_robot_listener = self.create_subscription(String, f'/add_robot', self.add_robot_listener_callback, 10)


    def add_robot_listener_callback(self, msg):
        msg_data = msg.data.split()
        name = msg_data[0]
        x = float(msg_data[1])
        y = float(msg_data[2])
        yaw = float(msg_data[3])

        self.robot_batery[name]=1.0
        self.robot_batery_publisher[name] = self.create_publisher(Float64, f'/model/{name}/batery', 10)
        self.robot_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/pose',
            lambda msg_g, nm=name: self.pose_listener_callback(msg_g, nm),
            10
        )
        self.robot_list[name] = {"x": x, "y": y, "Y": yaw}

        return

    
    def safe_destroy_callback(self):
        name = self.current_name_for_deletion

        self.destroy_publisher(self.robot_batery_publisher.pop(name))
        self.destroy_subscription(self.robot_pose_listener.pop(name))

        del self.robot_batery[name]
        del self.robot_list[name]

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
        distance = np.sqrt((self.robot_list[nm]["x"] - msg.poses[0].position.x) ** 2 + (self.robot_list[nm]["y"] - msg.poses[0].position.y) ** 2)

        self.robot_list[nm]["x"] = msg.poses[0].position.x
        self.robot_list[nm]["y"] = msg.poses[0].position.y

        q = msg.poses[0].orientation

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        theta_angle = np.arctan2(t3, t4)

        self.robot_list[nm]["Y"] = theta_angle

        self.robot_batery[nm] += - 0.001 * distance

        new_msg = Float64()

        new_msg.data = self.robot_batery[nm]

        self.robot_batery_publisher[nm].publish(new_msg)

        if self.robot_batery[nm] < 0.1:
            str_msg = String()
            str_msg.data = 'LowBattery'
            self.publisher_control.publish(str_msg)

    
    def send_pose_request(self, model_name, x, y, z, R, P, Y):
        req = SetEntityPose.Request()

        req.entity.name = model_name
        req.entity.type = 2

        cr = np.cos(R * 0.5)
        sr = np.sin(R * 0.5)
        cp = np.cos(P * 0.5)
        sp = np.sin(P * 0.5)
        cy = np.cos(Y * 0.5)
        sy = np.sin(Y * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        req.pose = pose

        future = self.client_set_pose.call_async(req)
        future.add_done_callback(self.callback)

    
    def send_spawn_static_request(self, x, y):
        req = SpawnEntity.Request()

        self.cylinder_id += 1
        name = 'cylinder' + str(self.cylinder_id)

        req.entity_factory.name = name

        with open('cylinder.sdf', 'r') as f:
            sdf_content = f.read()

        req.entity_factory.sdf = sdf_content

        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y

        future = self.client_spawn.call_async(req)
        future.add_done_callback(self.callback)


    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Результат вызова: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Сервис завершился с ошибкой: {e}')
    

    def clock_listener_callback(self, msg):
        if self.start_sim_time == None:
            self.start_sim_time = [msg.clock.sec, msg.clock.nanosec]
            return

        if (msg.clock.sec - self.start_sim_time[0] > self.sim_duration or (msg.clock.sec - self.start_sim_time[0] == self.sim_duration and msg.clock.nanosec > self.start_sim_time[1])):
            # self.send_request(True)
            raise SystemExit
        
        for dynamic_obstacle in self.dynamic_obstacles:
            test = 0
            # print(dynamic_obstacle)
        
        if len(self.event_list) > 0 and (self.event_list[0]['time'] < msg.clock.sec - self.start_sim_time[0] or (self.event_list[0]['time'] == msg.clock.sec - self.start_sim_time[0] and msg.clock.nanosec > self.start_sim_time[1])):
            event = self.event_list.popleft()
            
            print(msg.clock.sec, event['event'])

            if event['event'] == "Task":
                msg = String()

                if 'name' not in event.keys() or event['name'] not in self.robot_list.keys():
                    msg.data = f'{event['x']} {event['y']}'
                else:
                    msg.data = f'{event['x']} {event['y']} {event['name']}'
                
                self.task_publisher.publish(msg)
                return
            
            if event['event'] == "TreeFall":
                if event['name'] in self.model_dict:
                    if event['trigger'] == False:
                        model = self.model_dict[event['name']]
                        new_Y = model['Y'] + event['theta']
                        new_R = model['R']
                        new_P = model['P'] + 1.5
                        self.send_pose_request(event['name'], model['x'], model['y'], model['z'], new_R, new_P, new_Y)
                    else:
                        model = self.model_dict[event['name']]
                        self.dynamic_obstacles.append({'event': event['event'], 
                                                       'name': event['name'], 
                                                       'model': model, 
                                                       'theta': event['theta']})
                return
            
            if event['event'] == "SpawnStaticObstacle":
                if event['trigger'] == "Coordinates":
                    self.send_spawn_static_request(event['x'], event['y'])
                    return
                if event['trigger'] == "Dynamic":
                    self.dynamic_obstacles.append({'event': event['event'], 
                                                   'robot_name': event['robot_name'], 
                                                   'd': event['d'], 
                                                   'theta': event['theta']})
                return
            

def main(args=None):
    with open('config.json', 'r') as file:
        data = json.load(file)

    dic = {item['name']: {"x": item["x"], "y": item["y"], "Y": item["Y"]} for item in data['robot_list']}

    with open(data["world_config"], 'r') as file:
        world_config = json.load(file)

    tree = ET.parse(world_config["world_name"])
    root = tree.getroot()
    world = root.find('world')

    cut_name = ''

    if world is not None:
        cut_name = world.get('name')

    event_list = sorted(data['event_list'], key=lambda x: x['time'])

    model_dict = {}

    for model in root.findall(".//model"):
        name = model.get('name')
        pose = model.find('pose')
        if pose is not None:
            model_dict[name] = {
                "x": float(pose.text.split(' ')[0]), 
                "y": float(pose.text.split(' ')[1]),
                "z": float(pose.text.split(' ')[2]),
                "R": float(pose.text.split(' ')[3]),
                "P": float(pose.text.split(' ')[4]),
                "Y": float(pose.text.split(' ')[5]),
                }
            
    # print(model_dict)

    rclpy.init(args=args)
    node = EventGenerator(data["duration"], cut_name, event_list, model_dict, dic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()