from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import os
import xacro
import yaml

ARGUMENTS = [
    DeclareLaunchArgument('joints_value', default_value='',
                          description='joints_value [0.0, -0.97, 2.0, -2.56, -1.55, 0.0]'),
    DeclareLaunchArgument('robot_arm_name', default_value='arm_',
                          description='robot arm name'),
]


def generate_launch_description():

    
    # Launch args
    joints_value = LaunchConfiguration('joints_value') 
    robot_arm_name = LaunchConfiguration('robot_arm_name') 
    message = """ {
            'header': {
                'stamp': {
                'sec': 0,
                'nanosec': 0
                },
                'frame_id': ''
            },
            'joint_names': [
                'arm_shoulder_pan_joint', 
                'arm_shoulder_lift_joint',
                'arm_elbow_joint',
                'arm_wrist_1_joint',
                'arm_wrist_2_joint',
                'arm_wrist_3_joint'
            ],
            'points': [
                {
                'positions': [0.0, -0.97, 2.0, -2.56, -1.55, 0.0],
                'velocities': [],
                'accelerations': [],
                'effort': [],
                'time_from_start': {
                    'sec': 1,
                    'nanosec': 0
                }
                }
            ]
            }"""
    #TODO replace {arm_} by robot_arm_name parameter
    #msg_out = message.format(robot_arm_name,robot_arm_name,robot_arm_name,robot_arm_name,robot_arm_name,robot_arm_name)
    #print(msg_out)
    # Set initial joint position for robot.   This step is not needed for Humble 
    # In Humble, initial positions are taken from initial_positions.yaml and set by ros2 control plugin
    set_initial_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/arm_controller/joint_trajectory", # controller topic "/" + robot_name + "/arm_controller/joint_trajectory"
            "trajectory_msgs/msg/JointTrajectory",
            message,
        ],
        output="screen",
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(set_initial_pose) 
    return ld
