from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import tempfile


def generate_launch_description():
    config_name_arg = DeclareLaunchArgument(
        'config_name',
        default_value='single_drone_empty_room.yaml',
        description='Config file (in exploration_manager/config/) to load, one per scenario.',
    )
    config = PathJoinSubstitution([
        get_package_share_directory('exploration_manager'),
        'config', LaunchConfiguration('config_name')])
    # LKH's scratch dir, resolved from the installed package rather than
    # hardcoded to one checkout path (the solver writes its .atsp/.par problem
    # files here and mtsp_node reads them back from the same place).
    # Cartella di LAVORO del solver LKH: qui l'esplorazione scrive a ogni ciclo i
    # file .par/.atsp e il solver ci lascia il .tour. Prima era la cartella
    # resource/ del pacchetto, che con --symlink-install e' un collegamento ai
    # SORGENTI: ogni run riscriveva file del codice, con dentro percorsi
    # assoluti della macchina. Sono file temporanei, il loro posto e' /tmp.
    mtsp_dir = os.path.join(tempfile.gettempdir(), 'c2x_lkh')
    os.makedirs(mtsp_dir, exist_ok=True)

    return LaunchDescription([
        config_name_arg,
        Node(
            package='exploration_manager',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            # Override the scenario file's tsp/mtsp dirs so the node that writes
            # the LKH problem files and the solver nodes that read them always
            # agree on one path, whatever the config happens to say.
            parameters=[config, {
                'exploration.tsp_dir': mtsp_dir,
                'exploration.mtsp_dir': mtsp_dir,
            }],
        ),
        Node(
            package='lkh_mtsp_solver',
            executable='mtsp_node',
            name='tsp_solver_1',
            output='log',
            parameters=[{
                'exploration.drone_id': 1,
                'exploration.mtsp_dir': mtsp_dir,
                'exploration.problem_id': 1,
            }],
        ),
        Node(
            package='lkh_mtsp_solver',
            executable='mtsp_node',
            name='acvrp_solver_1',
            output='log',
            parameters=[{
                'exploration.drone_id': 1,
                'exploration.mtsp_dir': mtsp_dir,
                'exploration.problem_id': 2,
            }],
        ),
        Node(
            package='plan_manage',
            executable='traj_server',
            name='traj_server',
            output='screen',
            parameters=[{
                'traj_server.max_vel': 1.0,
            }],
        ),
    ])
