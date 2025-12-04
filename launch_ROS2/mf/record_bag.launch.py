import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown

def generate_launch_description():

    # Bag 저장 위치
    bag_dir = '/workspace/bags'

    # 기록할 토픽 리스트
    bag_topics = [
        # cam1
        '/cam1/color/image_raw/compressed',
        '/cam1/color/camera_info',
        '/cam1/depth/image_rect_raw/compressedDepth',
        '/cam1/depth/camera_info',

        # cam2
        '/cam2/color/image_raw/compressed',
        '/cam2/color/camera_info',
        '/cam2/depth/image_rect_raw/compressedDepth',
        '/cam2/depth/camera_info',
        '/cam2/extrinsics/depth_to_color',

        # Livox
        '/livox/lidar',
        '/livox/imu',

        # TF
        '/tf',
        '/tf_static'
    ]

    # ros2 bag 실행
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + bag_topics,
        cwd=bag_dir,
        output='screen',
        additional_env={
            'ROS_LOG_DIR': '/tmp/ros_logs'
        }
    )

    # 종료 시 ros2 bag 에 SIGINT 전달하여 정상 종료
    kill_bag_on_exit = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-2', 'ros2'],   # SIGINT
                    shell=True
                )
            ]
        )
    )

    return LaunchDescription([
        bag_record,
        kill_bag_on_exit
    ])

