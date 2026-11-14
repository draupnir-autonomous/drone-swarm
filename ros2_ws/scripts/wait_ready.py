#!/usr/bin/env python3
"""Attende che TUTTI i droni dichiarino di essere pronti, poi esce.

Sostituisce due meccanismi fragili nel banco di prova:

  * la lettura dei log con grep. Contava le occorrenze di "wait for trigger",
    che ogni drone ripete finche' non parte: un solo drone che lo stampava 62
    volte soddisfaceva la soglia di 3, e le run partivano con un drone mai
    avviato bruciando fino a 480 s prima che il problema fosse visibile.
  * il riscaldamento a tempo fisso di 30 s, che approssimava con un cronometro
    la condizione "la mappa e' pronta" invece di verificarla.

Qui si aspetta la condizione vera, dichiarata dal drone stesso, e in caso di
fallimento si dice QUALE drone manca e PERCHE'.

Uso:  wait_ready.py --drones 3 [--timeout 90] [--completa-dopo 5]
Esce 0 quando tutti sono pronti, 1 allo scadere del tempo.

--completa-dopo S (un drone solo, run_reale_exploration.sh, 7 novembre 2026): "pronto" anche
con ZERO frontiere se il recinto e' gia' tutto visto. In uno spazio piccolo (5 x 5 m, fase 8
del collaudo) il lidar vede tutto durante decollo e giro iniziale: le frontiere vanno a zero
prima del via, il drone non risultava mai pronto, il via non partiva e il sorvegliante faceva
atterrare dopo un minuto come per un guasto (prova generale 2026-11-07_1714). Per distinguere
"tutto visto" da "mappa che non riceve nulla" (anche li' zero frontiere) servono: giro iniziale
concluso (volo_pronto), mappa che riceve la nuvola (frontiere viste prima, o nuvole arrivate su
map_ros/cloud) e zero frontiere per S secondi. Col via l'FSM conclude ("CONCLUSA (niente piu' da
esplorare)", dopo i 15 s di tolleranza di min_explore_time_), rientra e atterra.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from c2x_msgs.msg import ExplorationReady


class ReadyWaiter(Node):
    def __init__(self, drone_num):
        super().__init__('c2x_ready_waiter')
        self.drone_num = drone_num
        self.state = {}
        # transient_local: se un drone e' diventato pronto prima che questo nodo
        # partisse, il suo ultimo messaggio arriva comunque.
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.subs = [
            self.create_subscription(
                ExplorationReady, f'/drone_{i}/exploration_ready',
                lambda msg, i=i: self.on_ready(i, msg), qos)
            for i in range(1, drone_num + 1)
        ]

        # --completa-dopo (vedi in testa): solo il drone 1.
        self.completa_dopo = 0.0
        self.max_frontiere = 0
        self.volo_pronto = False
        self.nuvole = 0
        self.completa_da = None

    def segui_mappa_completa(self, secondi):
        self.completa_dopo = secondi
        self.create_subscription(
            Bool, '/drone_1/volo_pronto', lambda m: setattr(self, 'volo_pronto', self.volo_pronto or m.data),
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # Contata e basta, senza decodificarla (raw): ~20 000 punti a 10 Hz.
        self.create_subscription(
            PointCloud2, '/drone_1/map_ros/cloud', lambda m: setattr(self, 'nuvole', self.nuvole + 1),
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)

    def on_ready(self, idx, msg):
        self.state[idx] = msg
        if idx == 1:
            self.max_frontiere = max(self.max_frontiere, msg.num_frontiers)

    def mappa_completa(self, ora):
        """Drone 1 col recinto gia' tutto visto da completa_dopo secondi (vedi in testa)."""
        m = self.state.get(1)
        ok = (self.completa_dopo > 0 and m is not None and m.odom_ok and m.num_frontiers == 0
              and self.volo_pronto and (self.max_frontiere > 0 or self.nuvole >= 20))
        if not ok:
            self.completa_da = None
            return False
        if self.completa_da is None:
            self.completa_da = ora
        return ora - self.completa_da >= self.completa_dopo

    def all_ready(self):
        return (len(self.state) == self.drone_num
                and all(m.ready for m in self.state.values()))

    def report(self):
        lines = []
        for i in range(1, self.drone_num + 1):
            m = self.state.get(i)
            if m is None:
                lines.append(f"    drone {i}: NESSUNA CONFERMA (nodo non avviato?)")
            elif m.ready:
                lines.append(f"    drone {i}: pronto ({m.num_frontiers} frontiere)")
            else:
                lines.append(f"    drone {i}: NON pronto - {m.detail}"
                             f" (odometria={'si' if m.odom_ok else 'no'},"
                             f" mappa={'si' if m.map_ok else 'no'})")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drones', type=int, required=True)
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--completa-dopo', type=float, default=0.0,
                    help='un drone solo: pronto anche col recinto gia\' tutto visto da tanti s')
    args = ap.parse_args()
    if args.completa_dopo > 0 and args.drones != 1:
        sys.exit('--completa-dopo vale solo con --drones 1')

    rclpy.init()
    node = ReadyWaiter(args.drones)
    if args.completa_dopo > 0:
        node.segui_mappa_completa(args.completa_dopo)
    start = node.get_clock().now()
    last_log = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
            if node.all_ready():
                print(f"    OK - {args.drones}/{args.drones} droni pronti dopo {elapsed:.1f}s")
                print(node.report())
                return 0
            if node.mappa_completa(elapsed):
                print(f"    OK - recinto gia' tutto visto dopo il giro iniziale ({elapsed:.1f}s: zero "
                      f"frontiere, {node.max_frontiere} al massimo prima, {node.nuvole} nuvole in "
                      "mappa): via per concludere, rientro e atterraggio")
                return 0
            if elapsed - last_log >= 5.0:
                last_log = elapsed
                pronti = sum(1 for m in node.state.values() if m.ready)
                print(f"    ...{pronti}/{args.drones} pronti ({elapsed:.0f}s)")
            if elapsed > args.timeout:
                print(f"ERRORE: non tutti i droni pronti entro {args.timeout:.0f}s")
                print(node.report())
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
