import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


# gets paths to the gazebo sdf in the share directory
path_prefix = get_package_share_directory('vehicleblue')
gazebosim_file = os.path.join(path_prefix, 'gazebosim', 'vehicle_blue.sdf')

def generate_launch_description():
    gazebosim_node = ExecuteProcess(
        cmd=["$(which gz) sim", gazebosim_file],
        shell=True
    )

    navigator_node = Node(
        package="vehicleblue",
        executable="navigator",
        name="navigator"
    )

    return LaunchDescription([
        gazebosim_node,
        navigator_node
    ])