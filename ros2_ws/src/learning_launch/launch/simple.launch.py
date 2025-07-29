from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类


def generate_launch_description():
    return LaunchDescription([
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_pub', # 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_sub', # 节点的可执行文件名
        ),
    ])