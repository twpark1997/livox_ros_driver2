import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():

    # 1. Livox driver node
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        env={
        'LD_LIBRARY_PATH': '/root/ros2_ws/install/livox_ros_driver2/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
        }
    )

    # 2. ros2 bag record 실행
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/livox/imu',
            '/livox/lidar'
        ],
        cwd='/workspace/bags',   # ← 작업 디렉토리 지정
        output='screen'
    )

    # 3. Launch 종료 시 ros2 bag 정상 종료
    kill_bag_on_exit = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-2', 'ros2'],  # SIGINT 전달
                    shell=True
                )
            ]
        )
    )

    return LaunchDescription([
        livox_driver,
        bag_record,
        kill_bag_on_exit
    ])

