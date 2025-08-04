import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():

    # 获取默认urdf路径
    urdf_package_path = get_package_share_directory('udf_tutorial')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', '01-myfirst.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'rviz', 'display_robot_model.rviz')
    # 声明一个urdf目录参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(default_urdf_path),
        description='Path to the urdf file'
    )
    # 通过文件路径，获取内容，并转换成参数，传入robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(["cat ",launch.substitutions.LaunchConfiguration('urdf_path')])
    rebot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{"robot_description":rebot_description_value}]
    )
        
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='robot_state_publisher',
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", default_rviz_config_path]
    )

    return launch.LaunchDescription([
       action_declare_arg_mode_path,
       action_robot_state_publisher,
       action_joint_state_publisher,
       action_rviz_node
    ])