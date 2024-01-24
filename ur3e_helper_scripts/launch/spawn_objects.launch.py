import os
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # spawn objects
    block_urdf = os.path.join(get_package_share_directory("ur3e_helper_scripts"), "urdf", "block.urdf")
    cube_urdf = os.path.join(get_package_share_directory("ur3e_helper_scripts"), "urdf", "cube.urdf")

    xyz_block = [("+5.6000", "-3.6000", "+1.0000"), ("+5.6000", "-3.7000", "+1.0000"), ("+5.6000", "-3.8000", "+1.0000"),
                 ("+5.5000", "-3.6000", "+1.0000"), ("+5.5000", "-3.7000", "+1.0000"), ("+5.5000", "-3.8000", "+1.0000")]
    rpy_block = [("+0.0000", "+0.0000", "+0.0000"), ("+1.5708", "+0.0000", "+0.0000"), ("+0.0000", "+1.5708", "+0.0000"),
                 ("+0.0000", "+0.0000", "+1.5708"), ("+1.5708", "+0.0000", "+1.5708"), ("+0.0000", "+1.5708", "+1.5708")]
    xyz_cube = [("+5.6000", "-3.5000", "+1.0000"), ("+5.5000", "-3.5000", "+1.0000")]
    rpy_cube = [("+0.0000", "+0.0000", "+0.0000"), ("+0.0000", "+0.0000", "+0.7854")]

    entity_id = int(str(time.time())[0:-8])

    spawn_nodes = []

    for n in range(len(xyz_block)):
        entity_id += 1
        spawn_nodes.append(
            Node(package='gazebo_ros', 
                executable='spawn_entity.py', 
                name='block_spawn_entity_' + str(entity_id),
                output='screen', 
                emulate_tty=True, 
                arguments=['-entity', "block_" + str(entity_id),
                            '-x', xyz_block[n][0], '-y', xyz_block[n][1], '-z', xyz_block[n][2],
                            '-R', rpy_block[n][0], '-P', rpy_block[n][1], '-Y', rpy_block[n][2],
                            '-file', block_urdf],
            )
        )

    for n in range(len(xyz_cube)):
        entity_id += 1
        spawn_nodes.append(
                Node(package='gazebo_ros', 
                    executable='spawn_entity.py', 
                    name='cube_spawn_entity_' + str(entity_id),
                    output='screen', 
                    emulate_tty=True, 
                    arguments=['-entity', "block_" + str(entity_id),
                                '-x', xyz_cube[n][0], '-y', xyz_cube[n][1], '-z', xyz_cube[n][2],
                                '-R', rpy_cube[n][0], '-P', rpy_cube[n][1], '-Y', rpy_cube[n][2],
                                '-file', cube_urdf],
                )
            )

    return LaunchDescription(spawn_nodes)

# End of Code