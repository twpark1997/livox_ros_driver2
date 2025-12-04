import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('livox_ros_driver2')
    launch_dir = os.path.join(pkg_dir, 'launch_ROS2', 'mf')

    # -------------------------------
    # 기존 include launch 3개 유지
    # -------------------------------
    mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mid360.launch.py')
        )
    )

    rs_multi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rs_multi_camera_launch.py')
        )
    )

    record_bag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'record_bag.launch.py')
        )
    )

    # -------------------------------
    # rqt를 독립 프로세스로 실행
    # -------------------------------
    perspective_file = os.path.join(
        os.path.dirname(__file__),
        "Default.perspective"
    )
    perspective_file = os.path.realpath(perspective_file)

    display_rqt = ExecuteProcess(
        cmd=[
            'rqt',
            '--perspective-file', perspective_file
        ],
        name='mf_rqt',
        output='screen',
        shell=False,
    )

    # -------------------------------
    # rqt 죽으면 전체 launch 종료
    # -------------------------------
    kill_on_display_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=display_rqt,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        mid360,
        rs_multi,
        record_bag,

        display_rqt,
        kill_on_display_exit,
    ])

