from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import json
import xml.etree.ElementTree as ET


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    with open('config.json', 'r') as file:
        data = json.load(file)

    with open(data["world_config"], 'r') as file:
        world_config = json.load(file)

    with open('/src/models/PetrSU_robot/og_model.sdf') as file:
        sdf_robot = file.read()

    sdf_robot = sdf_robot.replace('_UPDATE_RATE_', str(data['update_frequency']))
    sdf_robot = sdf_robot.replace('_STDDEV_', str(data['noise']))

    with open('/src/models/PetrSU_robot/model.sdf', "w") as file:
        file.write(sdf_robot)

    launchDescriptionArray = [
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/src/models'
        )
    ]

    tree = ET.parse(world_config["world_name"])
    root = tree.getroot()
    world = root.find('world')

    cut_name = ''

    if world is not None:
        cut_name = world.get('name')

    launchDescriptionArray.append(
        Node(
            executable='python3',
            arguments=['/src/robot_manager.py'],
            name='robot_manager',
            output='screen'
        )
    )

    launchDescriptionArray.append(
        Node(
            executable='python3',
            arguments=['/src/events_generator.py'],
            name='events_generator',
            output='screen'
        )
    )

    launchDescriptionArray.append(
        Node(
            executable='python3',
            arguments=['/src/telemetry.py'],
            name='telemetry',
            output='screen'
        )
    )

    launchDescriptionArray.append(
        Node(
            executable='python3',
            arguments=['/src/main.py'],
            name='main_api',
            output='screen'
        )
    )

    launchDescriptionArray.append(
        Node(
            executable='python3',
            arguments=['/src/noise_node.py'],
            name='noiseNode',
            output='screen'
        )
    )

    launchDescriptionArray.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'/world/{cut_name}/control@ros_gz_interfaces/srv/ControlWorld',
                f'/world/{cut_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                f'/world/{cut_name}/create@ros_gz_interfaces/srv/SpawnEntity',
                f'/world/{cut_name}/model/controllable_camera/link/base_link/sensor/gui_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                output='screen'
        )
    )

    for robot in data["robot_list"]:
        launchDescriptionArray.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-world', cut_name,
                           '-file', '/PetrSU_robot',
                           '-name', robot['name'],
                           '-x', str(robot['x']),
                           '-y', str(robot['y']),
                           '-Y', str(robot['Y'])],
                output='screen'
            )
        )
        launchDescriptionArray.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    f'/world/{cut_name}/model/{robot['name']}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    f'/model/{robot['name']}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
                    f'/model/{robot['name']}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
                    ],
                output='screen'
            )
        )

    launchDescriptionArray.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': f'-s {world_config["world_name"]} --seed {data["seed"]}',
                'on_exit_shutdown': 'True'
                }.items(),
        )
    )
    
    return LaunchDescription(launchDescriptionArray)