from http.server import executable
from launch_ros.actions import Node
import platform
import sys
import os

import launch
from typing import cast
from launch import LaunchDescription  # noqa: E402
from launch import LaunchIntrospector  # noqa: E402
from launch import LaunchService  # noqa: E402
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

import launch_ros.actions
import launch_ros.events



#TODO:
#start cam stream
#being able to subscribe to it from container
#calibration checker node that exits when a cal file is present
#integrate cal service
#extrinsic cal checker node that does the same thing as the intrinsic one
#integrate extrinsic cal service

#Goal for acceptance test:
#Test if intrinsic camera calibration is saved locally
#if not: 
# 1) wait for camera calibration parameters to be sent over a service call
# 2) Don't launch or publish to pose topic until recieved
# 3) Once they are recieved, save the file locally and publish the parameters to some topic for the other pose estimation nodes to launch
# 4) Update camera status in diagnostic_msgs?
#If the params are saved locally:
# 1) start up other pose estimation topics immediately
#During this entire time...
# The CSI cam should be publish its raw images to some topic since it is required for cal
# In the future: need to run some tests to ensure that the camera works before we attempt to stream its feed

#to see images, use image_tools
#$ ros2 run image_tools showimage --ros-args --remap image:=/visionSensorData
#viewing transforms:
#ros2 run rviz2 rviz2
# or view frames
#ros2 run tf2_tools view_frames

def generate_launch_description():

    launch.logging.launch_config.log_handler_factory = \
        lambda path, encoding=None: launch.logging.handlers.RotatingFileHandler(
            path, maxBytes=1024, backupCount=3, encoding=encoding)

    # Any number of actions can optionally be given to the constructor of LaunchDescription.
    # Or actions/entities can be added after creating the LaunchDescription.
    #user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'
    ld = LaunchDescription([
        launch.actions.LogInfo(msg='Beginning Launch of ipd')
    ])

    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        # this is the action     ^              and this, the event handler ^
        on_stdout=on_output,
        on_stderr=on_output,
    )))

    # Unset launch prefix to prevent other process from getting this setting.
    ld.add_action(launch.actions.SetLaunchConfiguration('launch-prefix', ''))

    #NEED INFO FROM SAM AND ALEX
    #ENTRYPOINT NAME (executable, setup.py)
    #PACKAGE NAME (package)
    #ACTUAL NODE NAME (in src)

    config_ext = os.path.join(
        '/home',
        'ext_cal.yaml'
    )

    cam_feed = Node(
        name = 'ipd_rawimg_pub',
        package = 'trimble_ipd',
        executable = 'ipd_rawimg_pub',
        output = 'screen'
    )

    cal_checker_node = Node(
        name = 'ipd_cal_checker',
        package = 'ipd_calib',
        executable = 'ipd_cal_checker',
        output = 'screen'
    )

    ext_checker_node = Node(
        name = 'ipd_ext_checker',
        package = 'ipd_calib',
        executable = 'ipd_ext_checker',
        output = 'screen'
    )
    pose_estimation_node = Node(
        name = 'ipd_pose_estimator',
        package = 'trimble_ipd',
        executable = 'ipd_pose_estimator',
    )

    int_cal_service = Node(
        name = 'set_int_node',
        package = 'set_int_cal',
        executable = 'set_int_node'
    )

    ext_cal_service = Node(
        name = 'set_ext_node',
        package = 'set_ext_cal',
        executable = 'set_ext_node'
    )

    static_ext_broadcaster = Node(
        name = 'static_tf2_py',
        executable = 'static_tf2_py',
        package = "static_tf2_py",
        #parameters = [config_ext]
    )

    cal_exit_event_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action = cal_checker_node,
            on_exit = [
                LogInfo(msg='Calibration parameters found'),
                #EmitEvent(event=Shutdown(reason='test done')),
                ext_checker_node
            ]
        )
    )

    ext_exit_event_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action = ext_checker_node,
            on_exit = [
                LogInfo(msg='Extrinsic calibration parameters found'),
                pose_estimation_node,
                static_ext_broadcaster,
                #EmitEvent(event=Shutdown(reason='test done')),
            ]
        )
    )

    #add event handlers
    ld.add_action(cal_exit_event_handler)
    ld.add_action(ext_exit_event_handler)

    #start the nodes
    ld.add_action(cam_feed)
    ld.add_action(int_cal_service)
    ld.add_action(ext_cal_service)
    ld.add_action(cal_checker_node)
    
    print('Starting introspection of launch description...')
    print('')

    # print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(debug=True)  # Use this instead to get more debug messages.
    # # ls = LaunchService()
    # ls.include_launch_description(ld)
    return ld


# if __name__ == '__main__':
#     sys.exit(main())