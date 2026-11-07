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

Uso:  wait_ready.py --drones 3 [--timeout 90]
Esce 0 quando tutti sono pronti, 1 allo scadere del tempo.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

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

    def on_ready(self, idx, msg):
        self.state[idx] = msg

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
    args = ap.parse_args()

    rclpy.init()
    node = ReadyWaiter(args.drones)
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
