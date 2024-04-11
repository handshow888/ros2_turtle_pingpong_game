from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    runturtlesim = Node(
        package="runturtlesim",
        executable="runturtlesim",
        output="screen"
    )
    spawn = Node(
        package="spawn",
        executable="spawn",
        output="screen"
    )
    '''
    turtle = Node(
        package="turtle",
        executable="turtle",
        output="screen"
    )
'''
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([spawn,runturtlesim])
    # 返回让ROS2根据launch描述执行节点
    return launch_description