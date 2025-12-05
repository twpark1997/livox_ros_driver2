import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown


def generate_launch_description():

    # 현재 launch 파일이 있는 폴더
    this_dir = Path(os.path.dirname(__file__))

    # qos.yaml이 이 폴더에 있다고 가정
    qos_file_path = this_dir / 'qos.yaml'

    if not qos_file_path.exists():
        raise FileNotFoundError(f"qos.yaml not found: {qos_file_path}")

    # Bag 저장 경로
    bag_dir = Path('/workspace/bags')
    bag_dir.mkdir(parents=True, exist_ok=True)

    # MCAP storage_config.yaml 경로
    storage_config_path = bag_dir / 'storage_config.yaml'

    # 파일이 없으면 자동 생성
    if not storage_config_path.exists():
        storage_config_path.write_text(
            "mcap:\n"
            "  compression: 'Zstd'\n"
            "  compression_level: 1\n"
            "  include_metadata: true\n"
        )

    # 기록할 ROS2 토픽 리스트
    bag_topics = [
        # cam1
        '/cam1/color/image_raw/compressed',
        '/cam1/color/camera_info',
        '/cam1/depth/image_rect_raw/compressedDepth',
        '/cam1/depth/camera_info',

        # cam2
        '/cam2/color/image_rect_raw/compressed', #d405
        '/cam2/color/camera_info',
        '/cam2/depth/image_rect_raw/compressedDepth',
        '/cam2/depth/camera_info',
        #'/cam2/extrinsics/depth_to_color',
        #'/cam2/aligned_depth_to_color/image_raw/compressedDepth',
        #'/cam2/aligned_depth_to_color/camera_info',

        # Livox
        '/livox/lidar',
        '/livox/imu',

        # TF
        '/tf',
        '/tf_static',
    ]

    # ros2 bag record 실행
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--storage-config', str(storage_config_path),
            '--qos-profile-overrides-path', str(qos_file_path),
            '--max-cache-size', '200000000',
            '--max-bag-size', '536870912',   # 512MB로 파일 분할
        ] + bag_topics,
        cwd=str(bag_dir),
        output='screen',
        additional_env={'ROS_LOG_DIR': '/tmp/ros_logs'}
    )

    # Shutdown 시 ros2 bag record 에 SIGINT 전달
    kill_bag_on_exit = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd="pkill -2 'ros2 bag record'",
                    shell=True
                )
            ]
        )
    )

    return LaunchDescription([
        bag_record,
        kill_bag_on_exit
    ])

