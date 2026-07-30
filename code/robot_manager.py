import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseArray
from collections import deque
import numpy as np
import json
import model_a
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import xml.etree.ElementTree as ET
from thetaStarPlanner import ThetaStarPlanner
from filterpy.kalman import KalmanFilter


class RobotFilter:
    def __init__(self, dt=0.1, noise=0.1, start_x=0.0, start_y=0.0, start_theta=0.0):
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        self.kf.x = np.array([start_x, start_y, start_theta, 0., 0., 0.])

        self.kf.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        self.kf.R = np.eye(3) * noise
        self.kf.Q = np.eye(6) * 0.01
        self.kf.P = np.eye(6) * 0.1
        
    def filter_step(self, x, y, theta):
        self.kf.predict()
        self.kf.update([x, y, theta])
        return self.kf.x[:3].flatten()


class RobotManager(Node):
    def __init__(self, 
                 cut_name='', 
                 robot_list=[], 
                 working_zone=[[0, 0], [10, 10]], 
                 update_frequency=1.0, 
                 path_tolerance=1.0,
                 linear_cap=4.0,
                 angular_k=40.0,
                 angular_cap=50.0,
                 robot_radius=2.0,
                 obstacles=([], []),
                 noise=0.02):
        super().__init__('robotManagerNode')

        self.cut_name = cut_name
        self.update_frequency = update_frequency
        self.noise = noise

        self.timer_to_delete = None
        self.current_name_for_deletion = None

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.bridge = CvBridge()

        robot_followers_dict = {}

        self.robot_cmd_vel = {}
        self.robot_noisy_pose_listener = {}
        self.robot_camera_listener = {}
        self.robot_filtered_pose_publisher = {}

        self.robot_filter = {}  

        for item in robot_list:
            name = item['name']
            robot_followers_dict[name] = model_a.RobotTelemetry(
                time=f'{0}.{0:03d}',
                idle_timer=None,
                status=0,
                pose=model_a.BasePose(
                    x=None, 
                    y=None,
                    theta=None
                ),
                noisy_pose=model_a.BasePose(
                    x=None, 
                    y=None,
                    theta=None
                ),
                velocity=model_a.BaseVelocity(
                    linear_x=0.0,
                    angular_z=0.0
                ),
                goal=None,
                path=deque(),
                img=None
            )
            self.robot_noisy_pose_listener[name] = self.create_subscription(
                PoseArray,
                f'/model/{name}/noisy_pose',
                lambda msg, nm=name: self.pose_listener_callback(msg, nm),
                10
            )
            self.robot_filtered_pose_publisher[name] = self.create_publisher(
                PoseArray, 
                f'/model/{name}/filtered_pose', 
                10
            )
            self.robot_cmd_vel[name] = self.create_publisher(
                Twist, 
                f'/model/{name}/cmd_vel', 
                10
            )
            self.robot_camera_listener[name] = self.create_subscription(
                Image,
                f'/world/{cut_name}/model/{name}/link/base_link/sensor/camera_front/image',
                lambda msg, nm=name: self.camera_listener_callback(msg, nm),
                10
            )
            self.robot_filter[name] = RobotFilter(1.0 / update_frequency, noise, item['x'], item['y'], item['Y'])

        self.manager_telemetry = model_a.ManagerTelemetry(
            time=f'{0}.{0:03d}', 
            robot_followers=robot_followers_dict, 
            tasks=[], 
            events=[], 
            path_graph="", 
            config=model_a.ManagerConfig(
                work_zone=model_a.BaseZone(
                    minx=np.min([working_zone[0][0], working_zone[1][0]]), 
                    maxx=np.max([working_zone[0][0], working_zone[1][0]]), 
                    miny=np.min([working_zone[0][1], working_zone[1][1]]), 
                    maxy=np.max([working_zone[0][1], working_zone[1][1]])
                ),
                update_task_time=1.0 / update_frequency,
                listen_names=[item['name'] for item in robot_list]
            )
        )

        self.idle_info_publisher = self.create_publisher(String, '/idle_info', 10)
        self.idle_info_timer = self.create_timer(1.0, self.publish_idle_info)

        self.task_subscription = self.create_subscription(String, '/add_task', self.task_listener_callback, 10)
        self.control_subscription = self.create_subscription(String, '/control', self.control_listener_callback, 10)

        self.path_tolerance = path_tolerance
        self.linear_cap = linear_cap
        self.angular_k = angular_k
        self.angular_cap = angular_cap

        grid_size = 2.0

        ox, oy = obstacles

        self.path_planner = ThetaStarPlanner(ox, oy, grid_size, robot_radius)

        self.remove_robot_listener = self.create_subscription(String, f'/remove_robot', self.remove_robot_listener_callback, 10)
        self.add_robot_listener = self.create_subscription(String, f'/add_robot', self.add_robot_listener_callback, 10)


    def add_robot_listener_callback(self, msg):
        msg_data = msg.data.split()
        name = msg_data[0]
        x = float(msg_data[1])
        y = float(msg_data[2])
        yaw = float(msg_data[3])

        self.manager_telemetry.robot_followers[name] = model_a.RobotTelemetry(
            time=f'{0}.{0:03d}',
            idle_timer=None,
            status=0,
            pose=model_a.BasePose(
                x=None, 
                y=None,
                theta=None
            ),
            noisy_pose=model_a.BasePose(
                x=None, 
                y=None,
                theta=None
            ),
            velocity=model_a.BaseVelocity(
                linear_x=0.0,
                angular_z=0.0
            ),
            goal=None,
            path=deque(),
            img=None
        )
        self.robot_filter[name] = RobotFilter(1.0 / self.update_frequency, self.noise, x, y, yaw)

        self.robot_filtered_pose_publisher[name] = self.create_publisher(
            PoseArray,
            f'/model/{name}/filtered_pose',
            10
        )
        self.robot_noisy_pose_listener[name] = self.create_subscription(
            PoseArray,
            f'/model/{name}/noisy_pose',
            lambda msg_g, nm=name: self.pose_listener_callback(msg_g, nm),
            10
        )
        self.robot_cmd_vel[name] = self.create_publisher(
            Twist, 
            f'/model/{name}/cmd_vel',
            10
        )
        self.robot_camera_listener[name] = self.create_subscription(
            Image,
            f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/image',
            lambda msg_g, nm=name: self.camera_listener_callback(msg_g, nm),
            10
        )

        return


    def safe_destroy_callback(self):
        name = self.current_name_for_deletion

        self.destroy_publisher(self.robot_filtered_pose_publisher.pop(name))
        self.destroy_publisher(self.robot_cmd_vel.pop(name))
        self.destroy_subscription(self.robot_noisy_pose_listener.pop(name))
        self.destroy_subscription(self.robot_camera_listener.pop(name))

        del self.manager_telemetry.robot_followers[name]
        del self.robot_filter[name]

        self.current_name_for_deletion = None

        if self.timer_to_delete:
            self.timer_to_delete.cancel()
            self.destroy_timer(self.timer_to_delete)


    def remove_robot_listener_callback(self, msg):
        name = msg.data

        if name not in self.robot_noisy_pose_listener.keys():
            return

        if self.current_name_for_deletion == None:
            self.current_name_for_deletion = name
        else:
            return
        
        self.timer_to_delete = None

        self.timer_to_delete = self.create_timer(0.0, self.safe_destroy_callback)

        return
    

    def control_listener_callback(self, msg):
        data = str(msg.data).split()

        if data[0] == 'None':
            return

        if data[0] == 'LowBattery':
            self.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.LowBattery
            return
        
        if data[0] == 'Stop':
            if self.manager_telemetry.robot_followers[data[1]].status == model_a.RobotStatus.Manual:
                self.manager_telemetry.robot_followers[data[1]].idle_timer = 0
            return
        
        if self.manager_telemetry.robot_followers[data[0]].status == model_a.RobotStatus.Manual:
            self.manager_telemetry.robot_followers[data[0]].idle_timer = 10
        
        if self.manager_telemetry.robot_followers[data[0]].status == model_a.RobotStatus.Idle or self.manager_telemetry.robot_followers[data[0]].status == model_a.RobotStatus.Moving:
            self.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.Manual
            self.manager_telemetry.robot_followers[data[0]].idle_timer = 10
            msg = String()
            msg.data = f'{data[0]} Manual'
            self.idle_info_publisher.publish(msg)

    def send_twist(self, name=None, linear_x=0, angular_z=0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.robot_cmd_vel[name].publish(twist)

        self.manager_telemetry.robot_followers[name].velocity.linear_x=float(linear_x)
        self.manager_telemetry.robot_followers[name].velocity.angular_z=float(angular_z)


    def task_listener_callback(self, msg):
        data = str(msg.data).split()

        if len(data) == 2:
            task_x = float(data[0])
            task_y = float(data[1])

            min_distance = float('inf')
            available_follower_name = None
            que = []

            for name, robot in self.manager_telemetry.robot_followers.items():
                if robot.status == model_a.RobotStatus.Idle:
                    pose_x = robot.pose.x
                    pose_y = robot.pose.y

                    rx, ry = self.path_planner.planning(pose_x, pose_y, task_x, task_y)

                    distance = np.sqrt((task_x - pose_x) ** 2 + (task_y - pose_y) ** 2)

                    if distance < min_distance and len(rx) > 0:
                        min_distance = distance
                        available_follower_name = name
                        que.clear()
                        for i in range(len(rx)):
                            que.append(model_a.BasePoint(x=rx[i], y=ry[i]))

            
            if available_follower_name != None:
                self.manager_telemetry.robot_followers[available_follower_name].status = model_a.RobotStatus.Moving
                self.manager_telemetry.robot_followers[available_follower_name].path = deque(que)
            else:
                print('Problem: unrichebale goal')

            return

        if len(data) == 3:
            task_x = float(data[0])
            task_y = float(data[1])
            available_follower_name = data[2]
            que = []

            if self.manager_telemetry.robot_followers[available_follower_name].status == model_a.RobotStatus.Idle:
                pose_x = self.manager_telemetry.robot_followers[available_follower_name].pose.x
                pose_y = self.manager_telemetry.robot_followers[available_follower_name].pose.y

                rx, ry = self.path_planner.planning(pose_x, pose_y, task_x, task_y)

                for i in range(len(rx)):
                    que.append(model_a.BasePoint(x=rx[i], y=ry[i]))

                self.manager_telemetry.robot_followers[available_follower_name].status = model_a.RobotStatus.Moving
                self.manager_telemetry.robot_followers[available_follower_name].path = deque(que)

                return
            else:
                print('Problem: unrichebale goal')


    def pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()

        last_time = float(self.manager_telemetry.robot_followers[name].time)

        self.manager_telemetry.robot_followers[name].time = f"{now_msg.sec}.{str(now_msg.nanosec)}"

        self.manager_telemetry.robot_followers[name].noisy_pose.x=msg.poses[0].position.x
        self.manager_telemetry.robot_followers[name].noisy_pose.y=msg.poses[0].position.y

        q = msg.poses[0].orientation

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        theta_angle = np.arctan2(t3, t4) 

        self.manager_telemetry.robot_followers[name].noisy_pose.theta = theta_angle

        measurement = [msg.poses[0].position.x, msg.poses[0].position.y, theta_angle]

        result = self.robot_filter[name].filter_step(*measurement)

        self.manager_telemetry.robot_followers[name].pose.x = result[0]
        self.manager_telemetry.robot_followers[name].pose.y = result[1]
        self.manager_telemetry.robot_followers[name].pose.theta = result[2]

        msg.poses[0].position.x = result[0]
        msg.poses[0].position.y = result[1]

        self.robot_filtered_pose_publisher[name].publish(msg)

        if self.manager_telemetry.robot_followers[name].status != model_a.RobotStatus.Moving:
            return
        
        if self.manager_telemetry.robot_followers[name].goal == None:
            if len(self.manager_telemetry.robot_followers[name].path) > 0:
                goal_point = self.manager_telemetry.robot_followers[name].path.popleft()
                self.manager_telemetry.robot_followers[name].goal = model_a.BasePoint(x=goal_point.x, y=goal_point.y)
            else:
                self.send_twist(name, 0, 0)
                self.manager_telemetry.robot_followers[name].status = model_a.RobotStatus.Idle
                return
            
        dx = self.manager_telemetry.robot_followers[name].goal.x - msg.poses[0].position.x
        dy = self.manager_telemetry.robot_followers[name].goal.y - msg.poses[0].position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)

        if distance < self.path_tolerance:
            self.manager_telemetry.robot_followers[name].goal = None
            return
        
        desired_angle = np.arctan2(dy, dx)
        angle_error = desired_angle - theta_angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        ang = self.angular_k * angle_error

        if ang > self.angular_cap:
            ang = self.angular_cap
        elif ang < -self.angular_cap:
            ang = -self.angular_cap

        line = self.linear_cap * (1.0 - (0.9 * np.min([np.abs(angle_error) / 1.0, 1.0])))

        self.send_twist(name, line, ang)

    
    def camera_listener_callback(self, msg, name):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        success, buffer = cv2.imencode('.jpg', cv_image)
        if success:
            self.manager_telemetry.robot_followers[name].img = base64.b64encode(buffer).decode('utf-8')
    

    def publish_idle_info(self):
        for name, _ in self.manager_telemetry.robot_followers.items():
            if self.manager_telemetry.robot_followers[name].status == model_a.RobotStatus.Manual:
                if self.manager_telemetry.robot_followers[name].idle_timer == None:
                    self.manager_telemetry.robot_followers[name].idle_timer = 10
                    return
                if self.manager_telemetry.robot_followers[name].idle_timer > 0:
                    self.manager_telemetry.robot_followers[name].idle_timer += -1
                    return
                if len(self.manager_telemetry.robot_followers[name].path) <= 0 and self.manager_telemetry.robot_followers[name].goal == None:
                    self.manager_telemetry.robot_followers[name].status = model_a.RobotStatus.Idle
                    self.manager_telemetry.robot_followers[name].idle_timer = None
                    self.send_twist(name, 0.0, 0.0)
                    return
                
                self.manager_telemetry.robot_followers[name].status = model_a.RobotStatus.Moving
                self.manager_telemetry.robot_followers[name].idle_timer = None
                return
            
            msg = String()
            msg.data = f'{name} {self.manager_telemetry.robot_followers[name].status.name}'
            self.idle_info_publisher.publish(msg)


def main(args=None):
    with open('config.json', 'r') as file:
        data = json.load(file)

    with open(data["world_config"], 'r') as file:
        world_config = json.load(file)

    tree = ET.parse(world_config["world_name"])
    root = tree.getroot()
    world = root.find('world')

    cut_name = ''

    if world is not None:
        cut_name = world.get('name')

    ox = world_config['obstacles']['ox']
    oy = world_config['obstacles']['oy']

    rclpy.init(args=args)
    node = RobotManager(cut_name, 
                        data['robot_list'], 
                        world_config["working_zone"], 
                        data["update_frequency"],
                        0.5, 2.5, 1.0, 2.0, 2.0,
                        (ox, oy), data['noise'])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()