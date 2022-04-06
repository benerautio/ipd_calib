from launch_ros.actions import Node
import platform
import sys

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
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

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

def main():

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

    # check_cal_exists = launch.actions.ExecuteProcess(cmd=[sys.executable,'./check_cam_cal.py'])
    # ld.add_action(check_cal_exists)

    #try lifecycle launch
 
    # use: 'zed' for "ZED" camera - 'zedm' for "ZED mini" camera
    #camera_model = 'zedm' 

    # URDF file to be loaded by Robot State Publisher
    #urdf = os.path.join(get_package_share_directory('stereolabs_zed'), 'urdf', camera_model + '.urdf')

    # ZED Configurations to be loaded by ZED Node
    #config_common = os.path.join(get_package_share_directory('stereolabs_zed'), 'config', 'common.yaml')

    #config_camera = os.path.join(get_package_share_directory('stereolabs_zed'), 'config', camera_model + '.yaml')

    # Set LOG format
    #os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

    # Launch Description
    #ld = launch.LaunchDescription()

    # Prepare the ZED node
    lc_node = LifecycleNode(
        namespace = '',        # must match the namespace in config -> YAML
        name = 'lc_talker',        # must match the node name in config -> YAML
        package = 'ipd_calib',
        executable = 'lifecycle_ex',
        output = 'screen',
    )

    # Prepare the Robot State Publisher node
    # rsp_node = Node(
    #     node_name = 'zed_state_publisher',
    #     package = 'robot_state_publisher',
    #     node_executable = 'robot_state_publisher',
    #     output = 'screen',
    #     arguments = [urdf, 'robot_description:=zed_description']
    # )

    # Make the ZED node take the 'configure' transition
    lc_test_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(lc_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the ZED node take the 'activate' transition
    lc_node_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(lc_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # When the ZED node reaches the 'inactive' state, make it take the 'activate' transition and start the Robot State Publisher
    lc_node_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = lc_node,
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'lc_node' reached the 'INACTIVE' state, start the 'Robot State Publisher' node and 'activating'." ),
                # Robot State Publisher
                #rsp_node,
                # Change State event ( inactive -> active )
                lc_node_activate_trans_event,
            ],
        )
    )

    # When the ZED node reaches the 'active' state, log a message.
    lc_node_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = lc_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'lc_node' reached the 'ACTIVE' state" ),
            ],
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action( lc_node_inactive_state_handler )
    ld.add_action( lc_node_active_state_handler )
    ld.add_action( lc_node )
    ld.add_action( lc_test_configure_trans_event)

    #camera intrinsic service: set_camera_info
    #pkg: 
    # def check_cal_exists_output_handler(event):
    #     target_str = 'True'
    #     print("check cal exists handler")
    #     if (~(target_str in event.text.decode())):
    #         return ld.add_action(check_cal_exists)
            #launch.
        #ld.add_action(launch.actions.ExecuteProcess(cmd=[sys.executable,'./check_cam_cal.py']))

    # ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
    #     target_action=check_cal_exists,
    #     on_stdout=check_cal_exists_output_handler,
    #     on_stderr=check_cal_exists_output_handler,
    # )))

    # ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnExecutionComplete(
    #     target_action
    # )))
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnShutdown(
        on_shutdown=[launch.actions.LogInfo(msg=[
            'Launch was asked to shutdown: ',
            launch.substitutions.LocalSubstitution('event.reason'),
        ])],
    )))

    #ld.add_action(launch.actions.ExecuteProcess(cmd=[sys.executable,'./check_cam_cal.py']))

    #ld.add_action(launch.actions.ExecuteProcess(cmd=[sys.executable,'./check_cam_cal.py']))


    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService(debug=True)  # Use this instead to get more debug messages.
    # ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    sys.exit(main())