#!/usr/bin/env python3
"""Camera della GUI di Gazebo che insegue il drone, con la traiettoria disegnata
(per le registrazioni dimostrative, vedi registra_demo_gazebo.sh).

Usa le librerie Python di Gazebo (gz.transport13, gz.msgs10): la posa del drone
arriva in continuo (/world/<mondo>/dynamic_pose/info) e la camera si aggiorna 30
volte al secondo con /gui/move_to/pose. La prima versione chiamava il comando gz
due volte al secondo: ogni chiamata ripartiva con la sua accelerazione e frenata e
nel video la camera andava a scatti.

* Camera: offset fisso nel mondo, rivolta al drone; il punto inseguito e' filtrato
  (costante di tempo di qualche secondo): la camera scivola invece di scattare.
  (/gui/follow risponde "true" ma su questa installazione la vista resta ferma.)
* Traiettoria: nastro arancione (TRIANGLE_STRIP, largo 10 cm) alla quota del drone,
  aggiunto a pezzi di mezzo metro con i marker della GUI (/marker, risponde Empty):
  solo grafica, non entra nella simulazione.
* Sfondo: con --riquadro, quattro pannelli neri attorno all'edificio coprono il
  piano del terreno (con lo sfondo nero della GUI l'ufficio resta su fondo nero).

Esce da sola quando la posa non arriva piu' da 10 s (simulazione chiusa).
Uso: camera_inseguimento.py [mondo] [modello] [--offset=-7,-7,10] [--filtro 3.0]
                            [--riquadro=xmin,xmax,ymin,ymax] [--senza_traiettoria]
Va lanciato SENZA l'ambiente ROS caricato (librerie di sistema di Gazebo).
"""
import argparse
import math
import sys
import threading
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

ARANCIO = (1.0, 0.42, 0.0)
ARANCIO_EMISSIVO = (0.55, 0.2, 0.0)


class Inseguitore:
    def __init__(self, mondo, modello):
        self.node = Node()
        self.modello = modello
        self.lock = threading.Lock()
        self.posa = None
        self.t_posa = 0.0
        self.node.subscribe(Pose_V, f'/world/{mondo}/dynamic_pose/info', self.su_pose)
        self.cam = None
        self.node.subscribe(Pose, '/gui/camera/pose', self.su_cam)

    def su_cam(self, msg):
        with self.lock:
            self.cam = (msg.position.x, msg.position.y, msg.position.z)

    def camera_reale(self):
        with self.lock:
            return self.cam

    def su_pose(self, msg):
        for p in msg.pose:
            if p.name == self.modello:
                with self.lock:
                    self.posa = (p.position.x, p.position.y, p.position.z)
                    self.t_posa = time.time()
                return

    def ultima(self):
        with self.lock:
            return self.posa, self.t_posa

    def muovi_camera(self, cam, bersaglio):
        dx, dy, dz = (b - c for b, c in zip(bersaglio, cam))
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy))   # positivo = guarda in basso
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        req = GUICamera()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = cam
        req.pose.orientation.w = cy * cp
        req.pose.orientation.x = -sy * sp
        req.pose.orientation.y = cy * sp
        req.pose.orientation.z = sy * cp
        self.node.request('/gui/move_to/pose', req, GUICamera, Boolean, 200)

    def marker(self, ns, ident, tipo, punti=(), colore=(0, 0, 0), emissivo=(0, 0, 0), pose=None, scala=None):
        m = Marker()
        m.ns, m.id = ns, ident
        m.action = Marker.ADD_MODIFY
        m.type = tipo
        m.visibility = Marker.GUI
        if pose:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pose
        if scala:
            m.scale.x, m.scale.y, m.scale.z = scala
        for campo, c in ((m.material.ambient, colore), (m.material.diffuse, colore),
                         (m.material.emissive, emissivo), (m.material.specular, (0, 0, 0))):
            campo.r, campo.g, campo.b, campo.a = c[0], c[1], c[2], 1.0
        for p in punti:
            q = m.point.add()
            q.x, q.y, q.z = p
        self.node.request('/marker', m, Marker, Empty, 300)


def nastro(punti, larghezza=0.10):
    """Vertici di un nastro orizzontale lungo i punti (per TRIANGLE_STRIP)."""
    out = []
    for i, p in enumerate(punti):
        a = punti[max(0, i - 1)]
        b = punti[min(len(punti) - 1, i + 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        ox, oy = -dy / n * larghezza / 2, dx / n * larghezza / 2
        out.append((p[0] + ox, p[1] + oy, p[2]))
        out.append((p[0] - ox, p[1] - oy, p[2]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mondo', nargs='?', default='cubicle_office')
    ap.add_argument('modello', nargs='?', default='x500_mid360_0')
    ap.add_argument('--offset', default='-7,-7,10')
    ap.add_argument('--filtro', type=float, default=1.5, help='costante di tempo (s) del punto inseguito')
    ap.add_argument('--riquadro', default='', help='xmin,xmax,ymin,ymax dell\'edificio: sfondo nero attorno')
    ap.add_argument('--senza_traiettoria', action='store_true')
    ap.add_argument('--frequenza', type=float, default=30.0)
    ap.add_argument('--guadagno', type=float, default=1.0,
                    help='quanto oltre il punto voluto si comanda, rispetto alla camera reale')
    a = ap.parse_args()
    off = tuple(float(v) for v in a.offset.split(','))
    ins = Inseguitore(a.mondo, a.modello)
    t0 = time.time()
    while ins.ultima()[0] is None:          # aspetta la simulazione (al massimo 2 minuti)
        if time.time() - t0 > 120:
            return 1
        time.sleep(0.2)
    if a.riquadro:
        x0, x1, y0, y1 = (float(v) for v in a.riquadro.split(','))
        L = 120.0
        pannelli = [((x0 - L / 2, 0.0), (L, 2 * L)), ((x1 + L / 2, 0.0), (L, 2 * L)),
                    (((x0 + x1) / 2, y0 - L / 2), (x1 - x0, L)), (((x0 + x1) / 2, y1 + L / 2), (x1 - x0, L))]
        for k, ((cx, cy), (sx, sy)) in enumerate(pannelli):
            ins.marker('sfondo', k, Marker.BOX, pose=(cx, cy, 0.01), scala=(sx, sy, 0.02))
    filtrato, t_prec = None, time.time()
    percorso, pezzo, ident = [], [], 0
    periodo = 1.0 / a.frequenza
    while True:
        inizio = time.time()
        p, t_p = ins.ultima()
        if inizio - t_p > 10.0:
            break
        alfa = 1.0 - math.exp(-(inizio - t_prec) / max(0.1, a.filtro))
        t_prec = inizio
        filtrato = p if filtrato is None else tuple(f + alfa * (v - f) for f, v in zip(filtrato, p))
        voluta = tuple(f + o for f, o in zip(filtrato, off))
        # /gui/move_to/pose anima ogni richiesta da capo (con accelerazione e
        # frenata): rinnovata 30 volte al secondo la camera percorreva solo una
        # piccola parte della strada e restava indietro (0.3 m/s contro 1 m/s del
        # drone). Si chiede quindi un punto oltre quello voluto, in proporzione a
        # quanto la camera reale (/gui/camera/pose) ne e' lontana.
        reale = ins.camera_reale()
        if reale is not None:
            comando = tuple(r + a.guadagno * (v - r) for r, v in zip(reale, voluta))
        else:
            comando = voluta
        # la camera guarda il drone dalla posizione comandata (stesso offset)
        mira = tuple(c - o for c, o in zip(comando, off))
        ins.muovi_camera(comando, mira)
        if not a.senza_traiettoria and p[2] > 0.4:
            if not percorso or math.dist(p[:2], percorso[-1][:2]) > 0.08:
                percorso.append(p)
                pezzo.append(p)
            lungo = sum(math.dist(pezzo[i][:2], pezzo[i + 1][:2]) for i in range(len(pezzo) - 1))
            if len(pezzo) >= 3 and lungo > 0.5:
                ins.marker('traiettoria', ident, Marker.TRIANGLE_STRIP, nastro(pezzo), ARANCIO, ARANCIO_EMISSIVO)
                ident += 1
                pezzo = [pezzo[-1]]
        time.sleep(max(0.0, periodo - (time.time() - inizio)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
