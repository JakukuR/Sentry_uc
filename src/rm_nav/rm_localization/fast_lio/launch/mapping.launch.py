import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    
        # Create LaunchConfigurations for the parameters with default values (not in config file)
    feature_extract_enable_param = LaunchConfiguration('feature_extract_enable',default='false') # bool
    point_filter_num_param = LaunchConfiguration('point_filter_num',default='3')                 # int
    max_iteration_param = LaunchConfiguration('max_iteration',default='3')                       # int
    filter_size_surf_param = LaunchConfiguration('filter_size_surf',default='0.5')               # double
    filter_size_map_param = LaunchConfiguration('filter_size_map',default='0.5')                 # double
    cube_side_length_param = LaunchConfiguration('cube_side_length',default='1000.0')            # double
    runtime_pos_log_enable_param =LaunchConfiguration('runtime_pos_log_enable',default='false')  # bool
    
    default_config_path = os.path.join(package_path, 'config', 'mid360.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    # lidar_tf = Node(
    #     name='lidar_tf',
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','-0.55','0','0','0','1','body','base_link']
    #     )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    # ld.add_action(lidar_tf)
    #ld.add_action(rviz_node)

    return ld
