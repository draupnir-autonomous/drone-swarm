#!/usr/bin/env python3
"""Registra i fotogrammi della telecamera di bordo durante una run.

Serve a produrre l'ingresso per una ricostruzione 3D densa a posteriori
(LingBot-Map e simili vogliono un video o una cartella di immagini). Non ha
nulla a che vedere con l'esplorazione: il lidar resta l'unico sensore che
alimenta mappa e pianificazione.

Deliberatamente AUTONOMO: avvia da se' il ponte Gazebo->ROS per la sola
telecamera richiesta e non tocca il file di lancio dell'esplorazione, cosi' le
run normali non pagano il costo di rendering se non stai registrando. Per lo
stesso motivo il sensore nel modello ha always_on=0: Gazebo lo attiva solo
quando qualcuno e' in ascolto.

Uso (a simulazione gia' avviata, in un altro terminale):
    record_camera.py --world office_small --drone 1 --out /tmp/volo_drone1

Poi si da' la cartella in pasto al modello di ricostruzione.
"""
import argparse
import os
import subprocess
import sys
import time


def gz_camera_topic(world: str, instance: int) -> str:
    """Topic che gz deriva per questa telecamera.

    Stessa logica del lidar: il modello non dichiara <topic>, cosi' gz ne
    costruisce uno che contiene il nome del modello - unico per istanza, perche'
    PX4 nomina l'istanza i "x500_mid360_<i>". Con un nome fisso i fotogrammi dei
    tre droni si interlaccerebbero nello stesso flusso.
    """
    return (f'/world/{world}/model/x500_mid360_{instance}'
            f'/link/camera_link/sensor/front_camera/image')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='office_small')
    ap.add_argument('--drone', type=int, default=1, help='1-based, come i namespace ROS')
    ap.add_argument('--out', default=None, help='cartella dei fotogrammi')
    ap.add_argument('--limit', type=int, default=0, help='massimo fotogrammi (0 = illimitato)')
    ap.add_argument('--fps', type=int, default=15,
                    help='fotogrammi al secondo del video finale; deve corrispondere '
                         'a update_rate della telecamera nel modello, altrimenti il '
                         'video scorre piu\' veloce o piu\' lento del volo reale')
    ap.add_argument('--no-video', action='store_true',
                    help='salva solo i fotogrammi, senza codificare il video')
    args = ap.parse_args()

    instance = args.drone - 1
    out = args.out or f'/tmp/c2x_camera_drone{args.drone}'
    os.makedirs(out, exist_ok=True)

    gz_topic = gz_camera_topic(args.world, instance)
    ros_topic = f'/drone_{args.drone}/camera/image'

    # Il ponte va avviato qui e non nel launch dell'esplorazione: attivarlo
    # sempre significherebbe far renderizzare a Gazebo tre flussi video anche
    # quando non servono, su una macchina che gia' fatica con tre PX4.
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         f'{gz_topic}@sensor_msgs/msg/Image[gz.msgs.Image',
         '--ros-args', '-r', f'{gz_topic}:={ros_topic}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'ponte avviato: {gz_topic}\n            -> {ros_topic}')
    time.sleep(2.0)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    import numpy as np
    # Pillow e non OpenCV: su questa macchina cv2 e' compilato contro numpy 1.x
    # mentre l'ambiente ha numpy 2.2, e l'import fallisce. Pillow non ha quel
    # vincolo e per scrivere JPEG basta e avanza.
    from PIL import Image as PilImage

    class Recorder(Node):
        def __init__(self):
            super().__init__('c2x_camera_recorder')
            self.n = 0
            self.last_report = time.time()
            self.create_subscription(Image, ros_topic, self.on_image, 10)

        def on_image(self, msg):
            # Conversione manuale invece di cv_bridge: una dipendenza in meno, e
            # il formato qui e' sempre quello dichiarato nel modello.
            if msg.encoding not in ('rgb8', 'bgr8'):
                self.get_logger().warn(f'formato non gestito: {msg.encoding}')
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                img = img[:, :, ::-1]   # Pillow vuole RGB
            PilImage.fromarray(img).save(
                os.path.join(out, f'{self.n:06d}.jpg'), quality=92)
            self.n += 1
            if time.time() - self.last_report > 5.0:
                self.last_report = time.time()
                print(f'  {self.n} fotogrammi in {out}')

    rclpy.init()
    node = Recorder()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.n == 0 and time.time() - node.last_report > 10.0:
                node.last_report = time.time()
                print('  ancora nessun fotogramma: la simulazione e\' avviata? '
                      f'il mondo e\' davvero "{args.world}"?')
            if args.limit and node.n >= args.limit:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\nregistrati {node.n} fotogrammi in {out}')
        node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()

    # I fotogrammi bastano al modello di ricostruzione, che accetta una cartella
    # di immagini. Il video serve a guardare cosa ha visto il drone, ed e' anche
    # l'altro formato d'ingresso accettato.
    if not args.no_video and node.n > 0:
        video = os.path.join(out, 'volo.mp4')
        cmd = ['ffmpeg', '-y', '-framerate', str(args.fps),
               '-i', os.path.join(out, '%06d.jpg'),
               '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', video]
        print(f'codifica video a {args.fps} fps...')
        r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if r.returncode == 0:
            mb = os.path.getsize(video) / 1e6
            print(f'video pronto: {video}  ({mb:.1f} MB, {node.n/args.fps:.0f} s)')
        else:
            print('codifica fallita:', r.stderr.decode()[-300:])
            print(f'i fotogrammi restano in {out} e bastano per la ricostruzione')
    return 0


if __name__ == '__main__':
    sys.exit(main())
