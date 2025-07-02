# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)
    
# FFMPEG Configuration to be loaded by ZED Node
default_config_ffmpeg = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'ffmpeg.yaml'
)

# Object Detection Configuration to be loaded by ZED Node
default_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'object_detection.yaml'
)
# Custom Object Detection Configuration to be loaded by ZED Node
default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'custom_object_detection.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def launch_setup(context, *args, **kwargs):
    return_array = []

    wrapper_dir = get_package_share_directory('zed_wrapper')    

    # Launch configuration variables
    node_log_type = LaunchConfiguration('node_log_type')

    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    stream_address = LaunchConfiguration('stream_address')
    stream_port = LaunchConfiguration('stream_port')

    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    config_ffmpeg = LaunchConfiguration('ffmpeg_config_path')
    object_detection_config_path = LaunchConfiguration('object_detection_config_path')
    custom_object_detection_config_path = LaunchConfiguration('custom_object_detection_config_path')

    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    custom_baseline = LaunchConfiguration('custom_baseline')

    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    custom_baseline_val = custom_baseline.perform(context)

    if(node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:  # 'screen' or 'log'
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
            }

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    if (camera_model_val == 'virtual' and float(custom_baseline_val) <= 0):
        return [
            LogInfo(msg="Please set a positive value for the 'custom_baseline' argument when using a 'virtual' Stereo Camera with two ZED X One devices."),
        ]
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    # Common configuration file
    if (camera_model_val == 'zed' or 
        camera_model_val == 'zedm' or 
        camera_model_val == 'zed2' or 
        camera_model_val == 'zed2i' or 
        camera_model_val == 'zedx' or 
        camera_model_val == 'zedxm' or
        camera_model_val == 'virtual'):
        config_common_path_val = default_config_common + '_stereo.yaml'
    else:
        config_common_path_val = default_config_common + '_mono.yaml'

    info = 'Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Camera configuration file
    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    info = 'Using camera configuration file: ' + config_camera_path
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # FFMPEG configuration file
    info = 'Using FFMPEG configuration file: ' + config_ffmpeg.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Object Detection configuration file
    info = 'Using Object Detection configuration file: ' + object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    # Custom Object Detection configuration file
    info = 'Using Custom Object Detection configuration file: ' + custom_object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # ROS parameters override file
    ros_params_override_path_val = ros_params_override_path.perform(context)
    if(ros_params_override_path_val != ''):
        info = 'Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Xacro command with options
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)
    xacro_command.append(' ')
    xacro_command.append('custom_baseline:=')
    xacro_command.append(custom_baseline_val)   
    if(enable_gnss_val=='true'):
        xacro_command.append(' ')
        xacro_command.append('enable_gnss:=true')
        xacro_command.append(' ')
        if(len(gnss_coords)==3):
            xacro_command.append('gnss_x:=')
            xacro_command.append(gnss_coords[0])
            xacro_command.append(' ')
            xacro_command.append('gnss_y:=')
            xacro_command.append(gnss_coords[1])
            xacro_command.append(' ')
            xacro_command.append('gnss_z:=')
            xacro_command.append(gnss_coords[2])
            xacro_command.append(' ')

    # Robot State Publisher node
    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output=node_log_effective,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            # Foxy does not support the isolated mode
            container_exec='component_container'
        else:
            container_exec='component_container_isolated'
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
                output=node_log_effective,
                composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            config_common_path_val,
            config_camera_path,
            config_ffmpeg,
            object_detection_config_path,
            custom_object_detection_config_path
    ]

    if( ros_params_override_path_val != ''):
        node_parameters.append(ros_params_override_path)

    node_parameters.append({
        'use_sim_time': use_sim_time,
        'simulation.sim_enabled': sim_mode,
        'simulation.sim_address': sim_address,
        'simulation.sim_port': sim_port,
        'stream.stream_address': stream_address,
        'stream.stream_port': stream_port,
        'general.camera_name': camera_name_val,
        'general.camera_model': camera_model_val,
        'svo.svo_path': svo_path,
        'svo.publish_svo_clock': publish_svo_clock,
        'general.serial_number': serial_number,
        'general.camera_id': camera_id,
        'pos_tracking.publish_tf': publish_tf,
        'pos_tracking.publish_map_tf': publish_map_tf,
        'sensors.publish_imu_tf': publish_imu_tf,
        'gnss_fusion.gnss_fusion_enabled': enable_gnss,
        'pos_tracking.enabled': True,
        'pos_tracking.imu_fusion': True,
        'pos_tracking.area_memory': True
    })

    if camera_model_val in ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']:
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    else:
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_log_type', default_value=TextSubstitution(text='both'),
            description='The log type of the node.',
            choices=['screen', 'log', 'both']),
        DeclareLaunchArgument('camera_name', default_value=TextSubstitution(text='zed'),
            description='The name of the camera.'),
        DeclareLaunchArgument('camera_model', description='[REQUIRED] The model of the camera.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
        DeclareLaunchArgument('container_name', default_value='', description='Container name.'),
        DeclareLaunchArgument('namespace', default_value='', description='Node namespace.'),
        DeclareLaunchArgument('node_name', default_value='zed_node', description='Node name.'),
        DeclareLaunchArgument('ros_params_override_path', default_value='', description='Path to override YAML.'),
        DeclareLaunchArgument('ffmpeg_config_path', default_value=TextSubstitution(text=default_config_ffmpeg), description='Path to FFMPEG config.'),
        DeclareLaunchArgument('object_detection_config_path', default_value=TextSubstitution(text=default_object_detection_config_path), description='Path to Object Detection config.'),
        DeclareLaunchArgument('custom_object_detection_config_path', default_value=TextSubstitution(text=default_custom_object_detection_config_path), description='Path to Custom Object Detection config.'),
        DeclareLaunchArgument('serial_number', default_value='0', description='Camera serial number.'),
        DeclareLaunchArgument('camera_id', default_value='-1', description='Camera ID.'),
        DeclareLaunchArgument('publish_urdf', default_value='true', description='Enable URDF publishing.', choices=['true', 'false']),
        DeclareLaunchArgument('publish_tf', default_value='true', description='Enable odom TF.', choices=['true', 'false']),
        DeclareLaunchArgument('publish_map_tf', default_value='true', description='Enable map TF.', choices=['true', 'false']),
        DeclareLaunchArgument('publish_imu_tf', default_value='false', description='Enable IMU TF.', choices=['true', 'false']),
        DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path), description='Path to xacro file.'),
        DeclareLaunchArgument('svo_path', default_value=TextSubstitution(text='live'), description='SVO input path.'),
        DeclareLaunchArgument('publish_svo_clock', default_value='false', description='Enable SVO clock.'),
        DeclareLaunchArgument('enable_gnss', default_value='false', description='Enable GNSS.', choices=['true', 'false']),
        DeclareLaunchArgument('gnss_antenna_offset', default_value='[]', description='GNSS antenna offset.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time.', choices=['true', 'false']),
        DeclareLaunchArgument('sim_mode', default_value='false', description='Enable simulation.', choices=['true', 'false']),
        DeclareLaunchArgument('sim_address', default_value='127.0.0.1', description='Simulator address.'),
        DeclareLaunchArgument('sim_port', default_value='30000', description='Simulator port.'),
        DeclareLaunchArgument('stream_address', default_value='', description='Stream address.'),
        DeclareLaunchArgument('stream_port', default_value='30000', description='Stream port.'),
        DeclareLaunchArgument('custom_baseline', default_value='0.0', description='Custom stereo baseline.'),
        OpaqueFunction(function=launch_setup)
    ])

