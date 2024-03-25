import os
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    share_dir = get_package_share_directory('steinicp_om')
    config_common_path = LaunchConfiguration('config_common_path')

    record_to_rosbag_arg = DeclareLaunchArgument(
        'record',
        default_value = 'False'
    )

    record_path_arg = DeclareLaunchArgument(
        'record_path',
        default_value = '/mnt/Data/SteinICPResult/'
    )

    record_bag_name_arg = DeclareLaunchArgument(
        'record_name',
        default_value = 'SteinICP_bag'
    )

    record_data = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([LaunchConfiguration('record')])
        ),
        cmd=['ros2', 'bag', 'record','-o', [LaunchConfiguration('record_path'), LaunchConfiguration('record_name')],
              '/stein_icp/odom_visualization', '/stein_icp/pose_visualization', '/stein_icp/trajectories',
              '/stein_icp/pose_with_covariance', '/stein_icp/scan_context', '/stein_icp/downsampled_cloud',
              '/stein_icp/source_cloud', '/stein_icp/original_cloud', '/stein_icp/deskewed_cloud',
              '/stein_icp/localmap_cloud', '/stein_icp/neighbourmap_cloud', '/stein_icp/particles', 'stein_icp/parameters',
              '/stein_icp/runtime'],
        output='screen'
    )
    
    default_config_SteinICP = os.path.join(
        get_package_share_directory('steinicp_om'),
        'config',
        'SteinICP_parameters.yaml'
    )
    
    default_config_ScanContext = os.path.join(
        get_package_share_directory('steinicp_om'),
        'config',
        'ScanContext_parameters.yaml'
    )
    
    steinicp_node = Node(
    package = 'steinicp_om',
    executable='steinicp_om_main',
    name = 'onlineFGO_test',
    namespace = 'steinicp',
    output = 'screen',
    parameters = [default_config_SteinICP, default_config_ScanContext]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', 'src/stein-icp/steinicp_om/config/SteinICP.rviz']
    )
    
    ld = LaunchDescription([
        steinicp_node,
        rviz_node,
        record_to_rosbag_arg,
        record_path_arg,
        record_bag_name_arg,
        record_data
    ])

    
    return ld
