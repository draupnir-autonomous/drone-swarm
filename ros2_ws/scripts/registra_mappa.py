#!/usr/bin/env python3
"""Raccoglie durante il volo i dati per la MAPPA REALISTICA (vedi ricostruisci_mappa.py).

Due flussi, entrambi gia' allineati al mondo:
  - le nuvole lidar che cloud_transform_bridge pubblica su <ns>/map_ros/cloud_mappa3d (la stessa nuvola senza gli altri droni),
    gia' posate con la posa del loro istante (FAST-LIO senza GPS, PX4
    interpolato col GPS). Si accumulano in una griglia fine (2 cm di default)
    invece dei voxel da 10 cm della mappa di esplorazione: e' la geometria da
    cui nasce la superficie. Per ogni voxel si tiene anche la direzione da cui
    e' stato visto, che serve a orientare le normali verso lo spazio libero;
  - i fotogrammi della telecamera di bordo, salvati come fotogrammi chiave
    (ogni kf_dist metri o kf_ang gradi di movimento) insieme alla posa della
    telecamera all'istante dello scatto, per colorare la superficie.

E' il ruolo che nel progetto Glob3R avrebbero le immagini; Glob3R non ha
ancora pubblicato il codice (ottobre 2026), quindi la geometria viene dal lidar
e le immagini danno il colore. La cartella prodotta resta utilizzabile da un
metodo basato su immagini quando sara' disponibile: fotogrammi + pose + intrinseci.

Si ferma con SIGINT/SIGTERM e salva tutto in --out:
  punti.npz              centri dei voxel, conteggi, direzione di osservazione
  fotogrammi/NNNNN.jpg   fotogrammi chiave (con piu' droni dN_NNNNN.jpg)
  fotogrammi/pose.json   pose delle telecamere (mondo ENU) e intrinseci

SCIAME (ottobre 2026): --drone accetta un elenco ("1,2,3") e UN SOLO registratore
accumula le nuvole e le foto di tutti i droni nella stessa griglia, cioe' produce
direttamente la mappa 3D FUSA dello sciame. Le nuvole sono gia' tutte nel mondo
condiviso (cloud_transform_bridge le posa con la posa di ogni drone piu' la sua
partenza), quindi fondere e' solo accumulare. Tre registratori separati non ci
stavano nella RAM del Jetson: uno costa ~1.8 GB a fine volo su cubicle_office, e le
zone viste da piu' droni sarebbero state tenute tre volte.

Uso (a simulazione avviata, con ROS caricato):
  registra_mappa.py --world office_small --drone 1 --out /tmp/c2x_mappa_realistica
  registra_mappa.py --world office_small --drone 1,2,3 --tempo_sim --out /tmp/c2x_sciame
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2

# Telecamera di bordo: stessa posa e ottica di c2x_description/x500_mid360.
CAM_T_BODY = np.array([0.10, 0.0, 0.12])
CAM_PITCH = 0.15            # rad, verso il basso
CAM_HFOV = 1.5708
LIDAR_OFFSET = np.array([0.0, 0.0, 0.25])


def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def slerp(q0, q1, s):
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)


class Flusso:
    """Stato di UN drone: storia delle pose e ultimo fotogramma chiave."""

    def __init__(self, drone):
        self.drone = drone
        self.storico = []          # (tempo nodo [s], posizione, quaternione wxyz)
        self.ultimo_kf = None
        self.scansioni = 0
        self.fotogrammi = 0


class Registratore(Node):
    def __init__(self, a):
        # tempo_sim: il nodo legge l'orologio della simulazione, come il resto dello
        # stack lanciato con tempo_sim:=true (vedi multi_drone_exploration.launch.py).
        # L'eta' dei fotogrammi si misura allora direttamente sul proprio orologio.
        super().__init__('registra_mappa', parameter_overrides=(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)] if a.tempo_sim else []))
        self.a = a
        self.vox = a.voxel
        self.lock = threading.Lock()
        self.flussi = {d: Flusso(d) for d in a.droni}
        self.clock = None          # ultimo tempo simulato [s] (solo senza tempo_sim)
        self.buffer = []           # (chiavi, punti, direzioni) delle scansioni non ancora fuse
        # Voxel gia' fusi, in pezzi di dimensione crescente (vedi fondi()).
        self.pezzi = []
        self.chiavi = np.zeros(0, np.int64)
        self.fotogrammi = []
        self.scansioni = 0
        os.makedirs(a.out, exist_ok=True)
        if a.foto:
            os.makedirs(os.path.join(a.out, 'fotogrammi'), exist_ok=True)

        q_aff = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST)
        for d, fl in self.flussi.items():
            ns = f'/drone_{d}'
            self.create_subscription(Odometry, f'{ns}/odom_world',
                                     lambda m, fl=fl: self.on_odom(m, fl), 50)
            self.create_subscription(PointCloud2, f'{ns}/map_ros/cloud_mappa3d',
                                     lambda m, fl=fl: self.on_cloud(m, fl), q_aff)
            # Telecamera solo con --foto: senza nessuno in ascolto Gazebo non la rende
            # nemmeno (always_on=0), e sul Jetson sono GPU e CPU risparmiate.
            if a.foto:
                self.create_subscription(Image, f'{ns}/camera/image',
                                         lambda m, fl=fl: self.on_image(m, fl), q_aff)
        # /clock serve solo a datare i fotogrammi (on_image) quando il nodo e' sull'orologio
        # di parete: senza --foto non ci si iscrive. Gazebo lo pubblica a ogni passo di
        # simulazione (centinaia di Hz), e ogni messaggio era una callback Python sul Jetson
        # gia' saturo. Con tempo_sim l'orologio del nodo E' gia' quello della simulazione.
        if a.foto and not a.tempo_sim:
            self.create_subscription(Clock, '/clock', self.on_clock,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def ora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ora_sim(self):
        """Tempo della simulazione [s], o None se non ancora noto."""
        if self.a.tempo_sim:
            t = self.ora()
            return t if t > 0.0 else None
        return self.clock

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(self, m, fl):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        fl.storico.append((self.ora(), np.array([p.x, p.y, p.z]),
                           np.array([q.w, q.x, q.y, q.z])))
        if len(fl.storico) > 400:
            del fl.storico[:100]

    @staticmethod
    def posa(st, t):
        """Posa del corpo all'istante t (orologio del nodo), interpolata sulla storia st."""
        if not st:
            return None
        if t >= st[-1][0]:
            return st[-1][1], st[-1][2]
        if t <= st[0][0]:
            return st[0][1], st[0][2]
        for i in range(len(st) - 1, 0, -1):
            a, b = st[i - 1], st[i]
            if a[0] <= t <= b[0]:
                s = (t - a[0]) / max(1e-6, b[0] - a[0])
                return a[1] + s * (b[1] - a[1]), slerp(a[2], b[2], s)
        return st[-1][1], st[-1][2]

    def on_cloud(self, m, fl):
        if not fl.storico:
            return
        campi = {f.name: f.offset for f in m.fields}
        n = m.width * m.height
        if n == 0 or not all(k in campi for k in 'xyz'):
            return
        raw = np.frombuffer(m.data, dtype=np.uint8).reshape(n, m.point_step)
        pts = np.stack([raw[:, campi[k]:campi[k] + 4].copy().view(np.float32).ravel()
                        for k in 'xyz'], axis=1).astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return
        # Origine del sensore: la nuvola arriva gia' posata, quindi basta la
        # posa piu' recente per sapere da che parte e' stata vista ogni cella.
        pos, q = fl.storico[-1][1], fl.storico[-1][2]
        sensore = pos + quat_to_rot(*q) @ LIDAR_OFFSET
        direz = sensore - pts
        direz /= np.maximum(np.linalg.norm(direz, axis=1, keepdims=True), 1e-6)
        k = np.floor(pts / self.vox).astype(np.int64) + (1 << 20)
        chiavi = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
        with self.lock:
            self.buffer.append((chiavi, pts, direz))
            self.scansioni += 1
            fl.scansioni += 1
            if len(self.buffer) >= 20:
                self.fondi()

    @staticmethod
    def riduci(ch, sm, dr, ct):
        # Accumulatori in PRECISIONE SINGOLA (sciame, ottobre 2026): con tre droni su
        # cubicle_office i voxel da 2 cm sono decine di milioni, e in doppia precisione (64 byte
        # a voxel, piu' i temporanei della fusione) il registratore arrivava a 3.4 GB e salvava in
        # swap con la RAM esaurita. In singola sono 36 byte: le somme delle coordinate restano
        # esatte al decimo di millimetro (al piu' qualche migliaio di colpi a ~15 m).
        uniche, inv = np.unique(ch, return_inverse=True)
        n = len(uniche)
        somme = np.column_stack([np.bincount(inv, weights=sm[:, k], minlength=n)
                                 for k in range(3)]).astype(np.float32)
        direz = np.column_stack([np.bincount(inv, weights=dr[:, k], minlength=n)
                                 for k in range(3)]).astype(np.float32)
        cont = np.bincount(inv, weights=ct, minlength=n).astype(np.int32)
        return uniche, somme, direz, cont

    def fondi(self, tutto=False):
        """Fonde le scansioni in attesa. Prima ogni 20 scansioni si rifondeva TUTTO
        l'accumulato (17 milioni di voxel a fine volo su cubicle_office): il costo
        cresceva col volo e sul Jetson prendeva quasi un core intero. Ora i voxel
        stanno in pezzi di dimensione crescente e si fondono due pezzi solo quando
        l'ultimo e' grande almeno meta' del precedente (come un contatore binario):
        stesso risultato, costo totale ~N log N invece di ~N^2/lotto."""
        if self.buffer:
            self.pezzi.append(self.riduci(
                np.concatenate([b[0] for b in self.buffer]),
                np.concatenate([b[1] for b in self.buffer]),
                np.concatenate([b[2] for b in self.buffer]),
                np.ones(sum(len(b[0]) for b in self.buffer))))
            self.buffer = []
        while len(self.pezzi) >= 2 and (tutto or 2 * len(self.pezzi[-1][0]) >= len(self.pezzi[-2][0])):
            b, a = self.pezzi.pop(), self.pezzi.pop()
            self.pezzi.append(self.riduci(*(np.concatenate([x, y]) for x, y in zip(a, b))))
        if tutto and self.pezzi:
            self.chiavi, self.somme, self.direzioni, self.conteggi = self.pezzi[0]

    def on_image(self, m, fl):
        ora_sim = self.ora_sim()
        if ora_sim is None or not fl.storico:
            return
        # Eta' del fotogramma sull'orologio della simulazione, riportata
        # sull'orologio del nodo (stessa logica di cloud_transform_bridge).
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # Tetto 2 s (era 0.5): il registratore gira a priorita' bassa (nice 10) su una
        # CPU satura, e un fotogramma elaborato con piu' di 0.5 s di ritardo prendeva
        # la posa di 0.5 s prima, cioe' quella sbagliata. La storia delle pose
        # (storico, 300-400 campioni) copre ben piu' di 2 s.
        eta = min(max(ora_sim - stamp, 0.0), 2.0)
        pc = self.posa(fl.storico, self.ora() - eta)
        if pc is None:
            return
        p, q = pc
        r_corpo = quat_to_rot(*q)
        r_cam = r_corpo @ rot_y(CAM_PITCH)
        t_cam = p + r_corpo @ CAM_T_BODY
        if fl.ultimo_kf is not None:
            t0, r0 = fl.ultimo_kf
            ang = math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(r0.T @ r_cam) - 1) / 2))))
            if np.linalg.norm(t_cam - t0) < self.a.kf_dist and ang < self.a.kf_ang:
                return
        if m.encoding not in ('rgb8', 'bgr8') or m.width == 0:
            return
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        img = img.reshape(m.height, m.width, 3)
        if m.encoding == 'bgr8':
            img = img[:, :, ::-1]
        from PIL import Image as PILImage
        # Un drone solo: nomi di prima (00012.jpg). Sciame: prefisso del drone.
        nome = (f'{fl.fotogrammi:05d}.jpg' if len(self.flussi) == 1
                else f'd{fl.drone}_{fl.fotogrammi:05d}.jpg')
        PILImage.fromarray(np.ascontiguousarray(img)).save(
            os.path.join(self.a.out, 'fotogrammi', nome), quality=92)
        fx = (m.width / 2.0) / math.tan(CAM_HFOV / 2.0)
        with self.lock:
            self.fotogrammi.append({'file': nome, 'drone': fl.drone, 't': t_cam.tolist(),
                                    'R': r_cam.tolist(), 'w': m.width, 'h': m.height,
                                    'fx': fx, 'fy': fx, 'cx': m.width / 2.0, 'cy': m.height / 2.0})
        fl.fotogrammi += 1
        fl.ultimo_kf = (t_cam, r_cam)

    def salva(self):
        with self.lock:
            self.fondi(tutto=True)
            if not self.pezzi:
                print('[registra_mappa] nessuna scansione registrata', flush=True)
                return
            centri = self.somme / np.maximum(self.conteggi, 1).astype(np.float32)[:, None]
            dirs = self.direzioni / np.maximum(np.linalg.norm(self.direzioni, axis=1,
                                                              keepdims=True), 1e-6)
            # Scrittura su un file temporaneo e poi rinomina (ottobre 2026): run_multi_exploration.sh
            # ricostruisce la mappa appena questo processo esce o dopo un'attesa massima, e con
            # 16 milioni di voxel savez_compressed sul Jetson superava l'attesa: la ricostruzione
            # leggeva un file a meta' e falliva in silenzio. Senza compressione e' molto piu'
            # veloce (il file resta in /tmp, non va nell'archivio della run).
            finale = os.path.join(self.a.out, 'punti.npz')
            parziale = finale + '.parziale.npz'
            np.savez(parziale, centri=centri.astype(np.float32), conteggi=self.conteggi,
                     direzioni=dirs.astype(np.float32), voxel=self.vox)
            os.replace(parziale, finale)
        per_drone = ', '.join(f'drone {d}: {fl.scansioni} scansioni, {fl.fotogrammi} foto'
                              for d, fl in self.flussi.items())
        if not self.a.foto:
            print(f'[registra_mappa] salvati {len(self.chiavi)} voxel da {self.scansioni} scansioni '
                  f'(telecamera spenta) in {self.a.out} ({per_drone})', flush=True)
            return
        with open(os.path.join(self.a.out, 'fotogrammi', 'pose.json'), 'w') as f:
            json.dump({'convenzione': 'mondo ENU; telecamera con asse x in avanti, y a '
                                      'sinistra, z in alto (convenzione Gazebo)',
                       'fotogrammi': self.fotogrammi}, f)
        print(f'[registra_mappa] salvati {len(self.chiavi)} voxel da {self.scansioni} scansioni '
              f'e {len(self.fotogrammi)} fotogrammi chiave in {self.a.out} ({per_drone})', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='office_small')
    # Uno o piu' droni ("1" oppure "1,2,3"): con piu' droni una sola mappa fusa.
    ap.add_argument('--drone', default='1')
    ap.add_argument('--out', default='/tmp/c2x_mappa_realistica')
    ap.add_argument('--voxel', type=float, default=0.02)
    ap.add_argument('--kf_dist', type=float, default=0.3)
    ap.add_argument('--kf_ang', type=float, default=10.0)
    # --foto: fotogrammi chiave della telecamera per colorare la mappa 3D (lo passa
    # run_multi_exploration.sh con FOTO=1, di serie dal 17 ottobre 2026). Senza, mappa
    # 3D grigia dalla sola geometria del lidar.
    ap.add_argument('--foto', action='store_true')
    # --tempo_sim: orologio della simulazione (lo stack gira con tempo_sim:=true).
    ap.add_argument('--tempo_sim', action='store_true')
    a = ap.parse_args()
    a.droni = [int(v) for v in str(a.drone).split(',') if v.strip()]

    # Ponte Gazebo -> ROS delle sole telecamere (uno per tutti i droni): il sensore ha
    # always_on=0, quindi Gazebo la rende solo finche' qualcuno e' in ascolto, cioe'
    # finche' gira questo registratore con --foto.
    ponte = None
    if a.foto:
        argomenti, rinomine = [], []
        for d in a.droni:
            gz_topic = (f'/world/{a.world}/model/x500_mid360_{d - 1}'
                        f'/link/camera_link/sensor/front_camera/image')
            argomenti.append(f'{gz_topic}@sensor_msgs/msg/Image[gz.msgs.Image')
            rinomine += ['-r', f'{gz_topic}:=/drone_{d}/camera/image']
        ponte = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *argomenti,
             '--ros-args', *rinomine],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init(args=['registra_mappa'])
    nodo = Registratore(a)
    fine = {'v': False}

    def stop(*_):
        fine['v'] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while not fine['v'] and rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.1)
    finally:
        nodo.salva()
        if ponte is not None:
            ponte.terminate()
            try:
                ponte.wait(timeout=5)
            except subprocess.TimeoutExpired:
                ponte.kill()
        nodo.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
