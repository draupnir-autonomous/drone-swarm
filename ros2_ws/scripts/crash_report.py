#!/usr/bin/env python3
"""Classifica un ribaltamento a partire dal registro di volo binario di PX4.

Nasce perche' i ribaltamenti hanno piu' di una causa e i messaggi di testo non
bastano a distinguerle: su questo progetto tre correzioni consecutive non hanno
spostato il numero di crash proprio perche' aggredivano un solo meccanismo.
I due osservati finora:

  SUOLO      la quota reale arriva a ridosso del pavimento prima del
             ribaltamento. Inclinarsi per accelerare toglie spinta verticale,
             quindi a quota di crociera bassa il cedimento porta a terra.
  ASSETTO    la quota resta buona ma l'inclinazione oscilla e cresce fino a
             rovesciare il velivolo, con velocita' ben oltre quella pianificata.

Uso:  crash_report.py <file.ulg> [...]
"""
import sys
import numpy as np
from pyulog import ULog


def quaternion_to_tilt(d):
    q = np.stack([d.data['q[0]'], d.data['q[1]'], d.data['q[2]'], d.data['q[3]']], axis=1)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return np.maximum(np.abs(roll), np.abs(pitch))


def report(path):
    u = ULog(path, ['vehicle_attitude', 'vehicle_local_position'])
    names = {d.name for d in u.data_list}
    if 'vehicle_attitude' not in names or 'vehicle_local_position' not in names:
        print(f"{path}: dati insufficienti")
        return

    att = [d for d in u.data_list if d.name == 'vehicle_attitude'][0]
    lp = [d for d in u.data_list if d.name == 'vehicle_local_position'][0]
    t0 = att.data['timestamp'][0]
    t = (att.data['timestamp'] - t0) / 1e6
    tilt = quaternion_to_tilt(att)
    tl = (lp.data['timestamp'] - t0) / 1e6
    alt = -lp.data['z']
    vel = np.hypot(lp.data['vx'], lp.data['vy'])

    if tilt.max() < 60.0:
        print(f"{path}: nessun ribaltamento (inclinazione max {tilt.max():.0f} gradi)")
        return

    # Primo superamento dei 60 gradi: l'istante in cui il velivolo e' andato,
    # non il picco a terra mentre rotola.
    i = int(np.argmax(tilt > 60.0))
    t_flip = t[i]

    # Finestra dei 2 s precedenti: quota minima raggiunta e velocita' massima.
    w = (tl > t_flip - 2.0) & (tl <= t_flip)
    alt_min = float(alt[w].min()) if w.any() else float('nan')
    vel_max = float(vel[w].max()) if w.any() else float('nan')
    # Quanto e' cresciuta l'inclinazione nei 2 s prima: distingue un urto
    # (salto istantaneo) da un'oscillazione che diverge.
    wa = (t > t_flip - 2.0) & (t <= t_flip)
    tilt_prima = float(tilt[wa][:max(1, int(wa.sum() * 0.5))].max()) if wa.any() else 0.0

    if alt_min < 0.35:
        causa = "SUOLO"
    elif tilt_prima > 40.0:
        causa = "ASSETTO"
    else:
        causa = "INDETERMINATA"

    print(f"{path}")
    print(f"  causa probabile : {causa}")
    print(f"  ribaltamento a  : t={t_flip:.1f}s")
    print(f"  quota minima 2s : {alt_min:.2f} m")
    print(f"  velocita max 2s : {vel_max:.2f} m/s")
    print(f"  inclinazione gia' presente 2s prima : {tilt_prima:.0f} gradi")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        report(p)
