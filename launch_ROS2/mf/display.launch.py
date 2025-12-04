import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 현재 launch 파일의 위치 기준으로 perspective 파일 경로 구성
    perspective_file = os.path.join(
        os.path.dirname(__file__),
        "Default.perspective"
    )

    # 절대경로 변환
    perspective_file = os.path.realpath(perspective_file)

    return LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='mf_rqt',
            arguments=[
                '--perspective-file', perspective_file
            ],
            output='screen'
        )
    ])

