from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """실제 하드웨어 (UNO Q) 연결 시 사용
    사전 준비: sudo apt install ros-jazzy-rosbridge-suite
    """
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
        ),
        Node(
            package='imu_turtle_demo',
            executable='imu_filter_node',
            name='imu_filter_node',
            output='screen',
        ),
        Node(
            package='imu_turtle_demo',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen',
            parameters=[{
                'linear_gain':  2.0,
                'angular_gain': 2.0,
                'danger_dist':  0.30,
                'warn_dist':    0.60,
            }]
        ),
        # App Lab 대시보드용 rosbridge (ws://localhost:9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),
    ])
