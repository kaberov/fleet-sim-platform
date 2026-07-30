import model_a
import cv2
import numpy as np
import base64
from collections import deque
from rclpy.node import Node
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseArray, Quaternion
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from sensor_msgs.msg import Image
import subprocess
import os
import signal


class ApiNode(Node):
    def __init__(self, 
                 img_wight = 500, 
                 img_height = 500, 
                 cut_name='', 
                 robot_list=[], 
                 working_zone=[[0, 0], [10, 10]], 
                 update_frequency=1.0,
                 obstacles=[[], []]):
        super().__init__('apiNode')

        self.robot_bridges = {}

        self.cut_name = cut_name

        self.timer_to_delete = None
        self.current_name_for_deletion = None

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.websocket = None

        self.repeats_list = []

        self.bridge = CvBridge()

        robot_followers_dict = {}

        self.robot_cmd_vel = {}
        self.robot_pose_listener = {}
        self.robot_camera_listener = {}
        self.robot_noisy_pose_listener = {}

        for name in robot_list:
            self.repeats_list.append(name)
            robot_followers_dict[name] = model_a.RobotTelemetry(
                time=f'0.000',
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

        manager_telemetry = model_a.ManagerTelemetry(
            time=f'0.000', 
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
                listen_names=robot_list
            )
        )

        self.api_telemetry = model_a.ApiTelemetry(
            current_follower_name=None, 
            follower_control=None,
            config=model_a.ApiConfig(
                img_wight=img_wight, 
                img_height=img_height
            ),
            manager_telemetry=manager_telemetry
        )

        self.api_img = None

        self.publisher_control = self.create_publisher(String, '/control', 10)
        self.publisher_task = self.create_publisher(String, '/add_task', 10)
        self.info_subscription = self.create_subscription(String, '/idle_info', self.idle_info_listener_callback, 10)

        self.gui_camera_listener = self.create_subscription(
                Image,
                f'/world/{cut_name}/model/controllable_camera/link/base_link/sensor/gui_camera/image',
                lambda msg: self.gui_camera_listener_callback(msg),
                10
            )

        self.pause_state = True

        # self.add_robot_publisher = self.create_publisher(String, '/add_robot', 10)
        self.remove_robot_publisher_control = self.create_publisher(String, '/remove_robot', 10)
        self.add_robot_publisher_control = self.create_publisher(String, '/add_robot', 10)

        self.client_control = self.create_client(ControlWorld, f'/world/{cut_name}/control')

        self.map_image = self.draw_map(obstacles[0], obstacles[1])

        self.client_spawn = self.create_client(SpawnEntity, f'/world/{cut_name}/create')

        while not self.client_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    
    def send_control_request(self):
        self.pause_state = not self.pause_state
        req = ControlWorld.Request()
        req.world_control.pause = self.pause_state
        future = self.client_control.call_async(req)
        future.add_done_callback(self.callback)
        return self.get_info()


    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Результат вызова: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Сервис завершился с ошибкой: {e}')


    def idle_info_listener_callback(self, msg):
        data = str(msg.data).split()

        if data[0] == 'None':
            return

        if data[0] not in self.api_telemetry.manager_telemetry.robot_followers.keys():
            return
        
        if data[1] == 'Manual':
            self.api_telemetry.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.Manual

        if data[1] == 'Idle':
            self.api_telemetry.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.Idle

        if data[1] == 'Moving':
            self.api_telemetry.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.Moving

        if data[1] == 'LowBattery':
            self.api_telemetry.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.LowBattery

        if data[1] == 'Waiting':
            self.api_telemetry.manager_telemetry.robot_followers[data[0]].status = model_a.RobotStatus.Waiting

    def task_listener_callback(self, msg):
        data = str(msg.data).split()

        task_x = float(data[0])
        task_y = float(data[1])

        return
    

    def pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()
        self.api_telemetry.manager_telemetry.robot_followers[name].time = f"{now_msg.sec}.{str(now_msg.nanosec)[:3]}"

        self.api_telemetry.manager_telemetry.robot_followers[name].pose.x=msg.poses[0].position.x
        self.api_telemetry.manager_telemetry.robot_followers[name].pose.y=msg.poses[0].position.y

        q = msg.poses[0].orientation

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        theta_angle = np.arctan2(t3, t4) 

        self.api_telemetry.manager_telemetry.robot_followers[name].pose.theta = theta_angle


    def noisy_pose_listener_callback(self, msg, name):
        now_msg = self.get_clock().now().to_msg()
        self.api_telemetry.manager_telemetry.robot_followers[name].time = f"{now_msg.sec}.{str(now_msg.nanosec)[:3]}"

        self.api_telemetry.manager_telemetry.robot_followers[name].noisy_pose.x=msg.poses[0].position.x
        self.api_telemetry.manager_telemetry.robot_followers[name].noisy_pose.y=msg.poses[0].position.y

        q = msg.poses[0].orientation

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        theta_angle = np.arctan2(t3, t4) 

        self.api_telemetry.manager_telemetry.robot_followers[name].noisy_pose.theta = theta_angle

    
    def camera_listener_callback(self, msg, name):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        success, buffer = cv2.imencode('.jpg', cv_image)
        if success:
            self.api_telemetry.manager_telemetry.robot_followers[name].img = base64.b64encode(buffer).decode('utf-8')


    def gui_camera_listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        success, buffer = cv2.imencode('.jpg', cv_image)
        if success:
            self.api_img = base64.b64encode(buffer).decode('utf-8')


    def draw_map(self, ox, oy):
        map_image = np.full((self.api_telemetry.config.img_wight, self.api_telemetry.config.img_height, 3), (170, 170, 170), np.uint8)

        if self.api_telemetry.manager_telemetry.config.work_zone.maxx is None:
            return None

        posx = self.api_telemetry.config.img_wight / (self.api_telemetry.manager_telemetry.config.work_zone.maxx - self.api_telemetry.manager_telemetry.config.work_zone.minx)
        posy = self.api_telemetry.config.img_height / (self.api_telemetry.manager_telemetry.config.work_zone.maxy - self.api_telemetry.manager_telemetry.config.work_zone.miny)

        for i in range(len(ox)):
            cv2.circle(map_image, (int((-self.api_telemetry.manager_telemetry.config.work_zone.minx + ox[i]) * posx),self.api_telemetry.config.img_height - int((-self.api_telemetry.manager_telemetry.config.work_zone.miny + oy[i]) * posy)), radius=2, color=(50, 50, 50), thickness=-1)

        return map_image


    def draw_cv_image(self):
        cv_image = self.map_image.copy()

        if self.api_telemetry.manager_telemetry.config.work_zone.maxx is None:
            return None

        posx = self.api_telemetry.config.img_wight / (self.api_telemetry.manager_telemetry.config.work_zone.maxx - self.api_telemetry.manager_telemetry.config.work_zone.minx)
        posy = self.api_telemetry.config.img_height / (self.api_telemetry.manager_telemetry.config.work_zone.maxy - self.api_telemetry.manager_telemetry.config.work_zone.miny)

        for t in self.api_telemetry.manager_telemetry.robot_followers.keys():
            if self.api_telemetry.manager_telemetry.robot_followers[t].pose.x is None:
                return None

            x = self.api_telemetry.manager_telemetry.robot_followers[t].pose.x
            y = self.api_telemetry.manager_telemetry.robot_followers[t].pose.y
            theta = self.api_telemetry.manager_telemetry.robot_followers[t].pose.theta
            cv2.fillPoly(cv_image,
                         pts=[np.array(np.apply_along_axis(lambda a: [
                             (a[0] * np.cos(theta) + a[1] * np.sin(theta)) * self.api_telemetry.config.img_wight * 0.02 + int(
                                 posx * (x - self.api_telemetry.manager_telemetry.config.work_zone.minx)),
                             (-a[0] * np.sin(theta) + a[1] * np.cos(
                                 theta)) * self.api_telemetry.config.img_height * 0.02 + self.api_telemetry.config.img_height - int(
                                 posy * (y - self.api_telemetry.manager_telemetry.config.work_zone.miny))],
                                                           1, np.array([[-1, 1], [0, 0], [-1, -1], [1, 0]])),
                                       np.int32)],
                         color=(0, 0, 0) if self.api_telemetry.current_follower_name == t else (85, 85, 85)
                         )

        return cv_image

    def get_info(self):
        now_msg = self.get_clock().now().to_msg()
        self.api_telemetry.manager_telemetry.time = f"{now_msg.sec}.{str(now_msg.nanosec)[:3]}"

        table = []

        (flag2, encodedImage2) = cv2.imencode(".jpg", cv2.imread('placeholder_image.jpg'))

        for name, robot in self.api_telemetry.manager_telemetry.robot_followers.items():
            if robot.pose.x is None or robot.noisy_pose.x is None:
                table.append({
                    "name": name, 
                    "status": str(robot.status.name), 
                    "real_pose": f"None",
                    "noise_pose": f"None",
                })
            else:
                table.append({
                    "name": name, 
                    "status": str(robot.status.name), 
                    "real_pose": f"{robot.pose.x:.2f} {robot.pose.y:.2f} {robot.pose.theta:.1f}",
                    "noise_pose": f"{robot.noisy_pose.x:.2f} {robot.noisy_pose.y:.2f} {robot.noisy_pose.theta:.1f}",
                })

        test_image = self.draw_cv_image()

        if test_image is None:
            return {
                "pause_state": self.pause_state,
                "time": self.api_telemetry.manager_telemetry.time,
                "current_follower_name": self.api_telemetry.current_follower_name,
                "size": len(self.api_telemetry.manager_telemetry.robot_followers),
                "goal": str(self.api_telemetry.manager_telemetry.robot_followers[self.api_telemetry.current_follower_name].goal) if self.api_telemetry.current_follower_name is not None else "None",
                "table": table,
                "img1": base64.b64encode(encodedImage2).decode('utf-8') if flag2 else "None",
                "img2": base64.b64encode(encodedImage2).decode('utf-8') if flag2 else "None",
                "img3": base64.b64encode(encodedImage2).decode('utf-8') if flag2 else "None",
            }
        
        (flag1, encodedImage1) = cv2.imencode(".jpg", test_image)

        camera_image = None

        if self.api_telemetry.current_follower_name != None:
            camera_image = self.api_telemetry.manager_telemetry.robot_followers[self.api_telemetry.current_follower_name].img

        return {
            "pause_state": self.pause_state,
            "time": self.api_telemetry.manager_telemetry.time,
            "current_follower_name": self.api_telemetry.current_follower_name,
            "size": len(self.api_telemetry.manager_telemetry.robot_followers),
            "goal": str(self.api_telemetry.manager_telemetry.robot_followers[self.api_telemetry.current_follower_name].goal) if self.api_telemetry.current_follower_name is not None else "None",
            "table": table,
            "img1": base64.b64encode(encodedImage1).decode('utf-8') if flag1 else "None",
            "img2": base64.b64encode(encodedImage2).decode('utf-8') if camera_image == None else camera_image,
            "img3": base64.b64encode(encodedImage2).decode('utf-8') if self.api_img == None else self.api_img,
        }


    def control(self, name: str, v: float, theta: float):
        if name not in self.api_telemetry.manager_telemetry.robot_followers.keys():
            # print('OK')
            return
        
        if self.api_telemetry.manager_telemetry.robot_followers[name].status != model_a.RobotStatus.Manual:
            return
        
        msg = String()
        msg.data = f'{name}'
        self.publisher_control.publish(msg)
        
        v = v * 2.5
        theta = theta * 2.0
    
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(theta)
        self.robot_cmd_vel[name].publish(twist)

        self.api_telemetry.manager_telemetry.robot_followers[name].velocity.linear_x=float(v)
        self.api_telemetry.manager_telemetry.robot_followers[name].velocity.angular_z=float(theta)

        return self.get_info()
    

    def stop_control(self, name: str):
        if name not in self.api_telemetry.manager_telemetry.robot_followers.keys():
            return
        
        if self.api_telemetry.manager_telemetry.robot_followers[name].status != model_a.RobotStatus.Manual:
            return
        
        twist = Twist()
        twist.linear.x = float(0.0)
        twist.angular.z = float(0.0)
        self.robot_cmd_vel[name].publish(twist)

        self.api_telemetry.manager_telemetry.robot_followers[name].velocity.linear_x=float(0.0)
        self.api_telemetry.manager_telemetry.robot_followers[name].velocity.angular_z=float(0.0)

        return self.get_info()


    def follow_node(self, name: str):
        if self.api_telemetry.current_follower_name is None or self.api_telemetry.current_follower_name in self.api_telemetry.manager_telemetry.robot_followers.keys():
            self.api_telemetry.current_follower_name = name
        else:
            self.api_telemetry.current_follower_name = None
        return self.get_info()

    
    def control_node(self, name: str):
        if name is None or name not in self.api_telemetry.manager_telemetry.robot_followers.keys():
            return self.get_info()
        
        msg = String()

        if self.api_telemetry.manager_telemetry.robot_followers[name].status == model_a.RobotStatus.Manual:
            msg.data = f'Stop {name}'
        else:
            msg.data = f'{name}'

        self.publisher_control.publish(msg)

        return self.get_info()
            
    
    def add_task(self, name: str, x: float, y: float):
        msg = String()

        if name not in self.api_telemetry.manager_telemetry.robot_followers.keys():
            msg.data = f'{x} {y}'
        else:
            msg.data = f'{x} {y} {name}'

        self.publisher_task.publish(msg)
        return self.get_info()


    def add_robot(self, name: str, x: float, y: float, Y: float):
        if name in self.robot_pose_listener.keys() or name == '' or name in self.repeats_list:
            return

        self.repeats_list.append(name)

        req = SpawnEntity.Request()

        req.entity_factory.name = name

        with open('model.sdf', 'r') as f:
            sdf_content = f.read()

        req.entity_factory.sdf = sdf_content

        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y

        cr = np.cos(0)
        sr = np.sin(0)
        cp = np.cos(0)
        sp = np.sin(0)
        cy = np.cos(Y * 0.5)
        sy = np.sin(Y * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        req.entity_factory.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        future = self.client_spawn.call_async(req)
        future.add_done_callback(self.callback)

        self.robot_bridges[name] = []

        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/model/{name}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist']
        p = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.robot_bridges[name].append(p)

        # time.sleep(5)
        # self.get_clock().sleep_for(Duration(seconds=2.0))

        str_msg = String()
        str_msg.data = f'{name} {x} {y} {Y}'

        self.add_robot_publisher_control.publish(str_msg)

        self.api_telemetry.manager_telemetry.robot_followers[name] = model_a.RobotTelemetry(
            time=f'0.000',
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
        self.robot_cmd_vel[name] = self.create_publisher(
            Twist, 
            f'/model/{name}/cmd_vel', 
            10
        )
        self.robot_camera_listener[name] = self.create_subscription(
            Image,
            f'/world/{self.cut_name}/model/{name}/link/base_link/sensor/camera_front/image',
            lambda msg, nm=name: self.camera_listener_callback(msg, nm),
            10
        )

        return self.get_info()


    def safe_destroy_callback(self):
        name = self.current_name_for_deletion
        if self.api_telemetry.current_follower_name == name:
            self.api_telemetry.current_follower_name = None

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.robot_cmd_vel[name].publish(twist)

        self.destroy_publisher(self.robot_cmd_vel.pop(name))
        self.destroy_subscription(self.robot_pose_listener.pop(name))
        self.destroy_subscription(self.robot_camera_listener.pop(name))
        self.destroy_subscription(self.robot_noisy_pose_listener.pop(name))

        del self.api_telemetry.manager_telemetry.robot_followers[name]

        msg = String()
        msg.data = f'{name}'
        self.remove_robot_publisher_control.publish(msg)

        if name in self.robot_bridges:
            for proc in self.robot_bridges.pop(name):
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

        self.current_name_for_deletion = None

        if self.timer_to_delete:
            self.timer_to_delete.cancel()
            self.destroy_timer(self.timer_to_delete)
    

    def remove_robot(self, name: str):
        if name not in self.robot_pose_listener.keys():
            return self.get_info()
        
        if self.current_name_for_deletion == None:
            self.current_name_for_deletion = name
        else:
            return self.get_info()

        self.timer_to_delete = None

        self.timer_to_delete = self.create_timer(0.0, self.safe_destroy_callback)

        return self.get_info()


def main(args=None):
    rclpy.init(args=args)
    node = ApiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()