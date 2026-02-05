"""
Spider Robot Bringup Launch File

Launches complete system including camera relay, servo control,
and optional teleoperation interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('spider_robot')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Enable ESP32-CAM camera streaming'
    )
    
    teleop_enabled_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Enable keyboard/joystick teleoperation'
    )
    
    i2c_addr_arg = DeclareLaunchArgument(
        'i2c_addr',
        default_value='0x40',
        description='PCA9685 I2C address'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_enabled = LaunchConfiguration('camera_enabled')
    teleop_enabled = LaunchConfiguration('teleop_enabled')
    i2c_addr = LaunchConfiguration('i2c_addr')
    
    # Camera relay node (ESP32-CAM interface)
    camera_relay_node = Node(
        package='spider_cam',
        executable='cam_relay',
        name='cam_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': '/spider/camera',
            'camera_url': 'http://192.168.4.1:81/stream',
            'frame_rate': 10,
            'enabled': camera_enabled
        }],
        condition=LaunchConfiguration('camera_enabled')
    )
    
    # Servo control bridge (Arduino/PCA9685 interface)
    servo_bridge_node = Node(
        package='spider_control',
        executable='servo_bridge',
        name='servo_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'i2c_addr': i2c_addr,
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'control_frequency': 50
        }]
    )
    
    # Gait controller node
    gait_controller_node = Node(
        package='spider_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'gait_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Kinematics node
    kinematics_node = Node(
        package='spider_control',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'kinematics.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Teleop keyboard node (optional)
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/spider/cmd_vel')
        ],
        condition=LaunchConfiguration('teleop_enabled')
    )
    
    # Robot state publisher (for visualization in RViz)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': os.path.join(pkg_dir, 'urdf', 'spider_robot.urdf')
        }]
    )
    
    # Joint state publisher (for servo positions)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        camera_enabled_arg,
        teleop_enabled_arg,
        i2c_addr_arg,
        
        # Nodes
        camera_relay_node,
        servo_bridge_node,
        gait_controller_node,
        kinematics_node,
        teleop_keyboard_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
