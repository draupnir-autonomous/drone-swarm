"""Stima per il collaudo sul drone VERO: FAST-LIO sul Mid-360 e il ponte della stima verso PX4.

Gli stessi nodi e topic della simulazione (/drone_1/lio/odometry, /drone_1/lio/salute,
/fmu/in/vehicle_visual_odometry), a tempo reale e senza nulla di Gazebo. Il driver Livox e
l'agente DDS li avvia avvia_collaudo.sh. Uso: ros2 launch collaudo_reale.launch.py

Se c'e' una telecamera USB (CAMERA_DEV, di serie /dev/video0; CAMERA=0 per non avviarla) la
pubblica su /camera/image_raw con cam2image di image_tools (gia' in ROS, OpenCV): 640x480 a
10 Hz. Per ora serve solo alla fase 1 del collaudo (la missione fa la mappa grigia).
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

QUI = os.path.dirname(os.path.abspath(__file__))


def telecamera():
    dev = os.environ.get('CAMERA_DEV', '/dev/video0')
    if os.environ.get('CAMERA', '1') != '1' or not os.path.exists(dev):
        return []
    num = dev.rstrip('0123456789')
    num = int(dev[len(num):]) if dev[len(num):] else 0
    return [Node(
        package='image_tools', executable='cam2image', name='telecamera', output='log',
        parameters=[{'device_id': num, 'width': 640, 'height': 480, 'frequency': 10.0,
                     'reliability': 'best_effort', 'frame_id': 'telecamera'}],
        remappings=[('image', '/camera/image_raw')],
    )]


def generate_launch_description():
    ns = '/drone_1'
    return LaunchDescription(telecamera() + [
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fast_lio', namespace=ns,
            output='log',
            parameters=[os.path.join(QUI, 'fast_lio_mid360.yaml')],
            remappings=[('/Odometry', f'{ns}/lio/odometry'),
                        ('/cloud_registered', f'{ns}/lio/cloud'),
                        ('/path', f'{ns}/lio/path'),
                        ('/lio_salute', f'{ns}/lio/salute')],
        ),
        # Stima di FAST-LIO dentro PX4 come visione esterna, con i controlli di salute
        # (atterraggio se i punti agganciati restano sotto 500 o se la stima diverge).
        Node(
            package='c2x_px4_bridge', executable='lio_to_px4_bridge', name='lio_to_px4_bridge',
            namespace=ns, output='log',
            parameters=[{'px4_ns': '', 'lio_topic': f'{ns}/lio/odometry',
                         'salute_topic': f'{ns}/lio/salute',
                         'salute_min_agganciati':
                             float(os.environ.get('SALUTE_MIN_AGGANCIATI', 500))}],
        ),
    ])
