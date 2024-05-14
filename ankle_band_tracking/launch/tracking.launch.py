import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LocalSubstitution

def generate_launch_description():

    package_name='ankle_band_tracking'

    ankle_band_tracking_share_dir = get_package_share_directory(package_name)
    yolo_param_file = os.path.join(ankle_band_tracking_share_dir, 'config', 'detector.yaml')
    follow_param_file = os.path.join(ankle_band_tracking_share_dir, 'config', 'follower.yaml')

    ### Lanch File ###
    realsense = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([os.path.join(
                   get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'
               )]),
               launch_arguments = {
                    'align_depth.enable': 'true'}.items(),
    )

    ### Node ###
    detector_node = Node(
        package=package_name,
        executable='detector',
        output='screen',
        parameters=[yolo_param_file]
    )

    follower_node = Node(
        package=package_name,
        executable='follower',
        remappings=[
            ('/yolo_detection/detector/bboxes', 'yolo_detection/detector/bboxes'),
        ],
        output='screen',
        parameters=[follow_param_file]
    )

    ### ExecuteProcess ###
    # rqt = ExecuteProcess(
    #     cmd=["rqt"], 
    #     output="screen",
    #     shell=True
    # )


    ### Event Handlers ###


    return LaunchDescription([

        realsense,
        detector_node,
        follower_node,
        #rqt,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])