import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import os

def generate_launch_description():

    urdf_package_path = get_package_share_directory('urdf_description')
    defaulturdf_path = os.path.join(urdf_package_path, 'urdf', 'first_robot.xacro')
    # 声明一个urdf目录参数first_robot.xacro
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name = 'model',
        default_value=defaulturdf_path,
        description='Path to the urdf file to be visualized'
    )
    default_rviz_config_path = os.path.join(urdf_package_path, 'rviz', 'default.rviz')

    # 通过文件路径，获取内容，并转换成参数，传入robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(["xacro ",launch.substitutions.LaunchConfiguration('model')])
    print("substitutions_command_result", substitutions_command_result)
    rebot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)
    print("rebot_description_value", rebot_description_value)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # 节点参数
        parameters=[{'robot_description':rebot_description_value}]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 命令行参数
        arguments=["-d", default_rviz_config_path]
    )

    # 返回所有要启动的节点描述
    return LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])