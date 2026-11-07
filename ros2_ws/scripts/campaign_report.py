#!/usr/bin/env python3
"""Diagnostica aggregata di una campagna: cerca cosa hanno IN COMUNE i guasti.

Nasce da un limite di metodo emerso lavorando su questo sistema: analizzando un
crash alla volta si trovano spiegazioni plausibili che le misure poi smentiscono
(nove correzioni tentate, sette smentite). Con piu' campioni si possono invece
confrontare le run finite male con quelle finite bene, ed e' il confronto - non
il singolo caso - a dire quale grandezza discrimina davvero.

Uso:  campaign_report.py [cartella]     (default /tmp/c2x_reliability)
"""
import glob
import os
import re
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    ULog = None


def tilt_series(att):
    q = np.stack([att.data['q[0]'], att.data['q[1]'], att.data['q[2]'], att.data['q[3]']], axis=1)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return np.maximum(np.abs(roll), np.abs(pitch))


def analizza_volo(path):
    """Estrae le grandezze di un volo, e se c'e' un ribaltamento le misura anche
    nei 3 s che lo precedono. Restituisce None se il registro non e' leggibile."""
    if ULog is None:
        return None
    try:
        u = ULog(path, ['vehicle_attitude', 'vehicle_local_position', 'trajectory_setpoint'])
    except Exception:
        return None
    nomi = {d.name for d in u.data_list}
    if 'vehicle_attitude' not in nomi or 'vehicle_local_position' not in nomi:
        return None
    att = [d for d in u.data_list if d.name == 'vehicle_attitude'][0]
    lp = [d for d in u.data_list if d.name == 'vehicle_local_position'][0]
    t0 = att.data['timestamp'][0]
    t = (att.data['timestamp'] - t0) / 1e6
    tilt = tilt_series(att)
    tl = (lp.data['timestamp'] - t0) / 1e6
    alt = -lp.data['z']
    vel = np.hypot(lp.data['vx'], lp.data['vy'])

    r = {
        'file': os.path.basename(path),
        'durata': float(t[-1]) if len(t) else 0.0,
        'quota_mediana': float(np.median(alt)),
        'vel_mediana': float(np.median(vel[vel > 0.05])) if (vel > 0.05).any() else 0.0,
        'vel_p95': float(np.percentile(vel, 95)),
        'fermo_pct': float(100 * np.mean(vel < 0.05)),
        'ribaltato': bool(tilt.max() > 60.0),
    }
    if r['ribaltato']:
        i = int(np.argmax(tilt > 60.0))
        tf = t[i]
        w = (tl > tf - 3.0) & (tl <= tf)
        wa = (t > tf - 3.0) & (t <= tf)
        r.update({
            't_crash': float(tf),
            'quota_min_3s': float(alt[w].min()) if w.any() else float('nan'),
            'vel_max_3s': float(vel[w].max()) if w.any() else float('nan'),
            'tilt_max_3s_prima': float(tilt[wa][:max(1, int(wa.sum() * 0.5))].max()) if wa.any() else 0.0,
            'frazione_volo': float(tf / t[-1]) if t[-1] > 0 else 0.0,
        })
    return r


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2x_reliability'
    print(f"=== diagnostica campagna: {base} ===\n")

    # --- esiti per run, dal riepilogo testuale ---
    ris = os.path.join(base, 'reliability.txt')
    crash_run, tot_run = set(), 0
    if os.path.exists(ris):
        blocco = open(ris, errors='ignore').read()
        for m in re.finditer(r"^(\S+) (\d+):\n(.*?)(?=^\S+ \d+:|\Z)", blocco, re.S | re.M):
            tot_run += 1
            if re.search(r"crash_fisici=([1-9])", m.group(3)):
                crash_run.add(int(m.group(2)))
    print(f"run analizzate: {tot_run}   con ribaltamento: {len(crash_run)} "
          f"({100 * len(crash_run) / max(1, tot_run):.0f}%)\n")

    # --- voli, separati fra chi si e' ribaltato e chi no ---
    voli = [v for v in (analizza_volo(p) for p in sorted(glob.glob(os.path.join(base, 'rel_ulog_*.ulg'))))
            if v]
    rott = [v for v in voli if v['ribaltato']]
    sani = [v for v in voli if not v['ribaltato']]
    print(f"voli letti: {len(voli)}   ribaltati: {len(rott)}   integri: {len(sani)}\n")

    if not rott:
        print("nessun ribaltamento da confrontare.")
    else:
        print("--- CONFRONTO: cosa distingue i voli finiti male ---")
        print(f"{'grandezza':28s} {'ribaltati':>12s} {'integri':>12s}")
        for chiave, etichetta in (('quota_mediana', 'quota mediana (m)'),
                                  ('vel_mediana', 'velocita in moto (m/s)'),
                                  ('vel_p95', 'velocita 95esimo (m/s)'),
                                  ('fermo_pct', 'tempo fermo (%)')):
            a = np.median([v[chiave] for v in rott])
            b = np.median([v[chiave] for v in sani]) if sani else float('nan')
            print(f"  {etichetta:26s} {a:12.2f} {b:12.2f}")

        print("\n--- I RIBALTAMENTI, uno per riga ---")
        print(f"{'file':22s} {'t':>7s} {'quota_min':>10s} {'vel_max':>8s} {'incl_prima':>11s} {'quando':>8s}")
        for v in sorted(rott, key=lambda x: x['t_crash']):
            print(f"  {v['file']:20s} {v['t_crash']:7.1f} {v['quota_min_3s']:10.2f} "
                  f"{v['vel_max_3s']:8.2f} {v['tilt_max_3s_prima']:11.0f} "
                  f"{100 * v['frazione_volo']:7.0f}%")

        suolo = sum(1 for v in rott if v['quota_min_3s'] < 0.35)
        print(f"\n  a ridosso del suolo (<0.35 m): {suolo}/{len(rott)}")
        veloci = sum(1 for v in rott if v['vel_max_3s'] > 2.0)
        print(f"  sopra 2 m/s nei 3 s prima:     {veloci}/{len(rott)}")
        presto = sum(1 for v in rott if v['frazione_volo'] < 0.25)
        print(f"  nel primo quarto del volo:     {presto}/{len(rott)}")

    # --- fermate di sicurezza e distanze di intervento ---
    dist = []
    for f in glob.glob(os.path.join(base, 'rel_expl_*.log')):
        dist += [float(x) for x in re.findall(r"PROXIMITY STOP: obstacle at ([0-9.]+)m",
                                              open(f, errors='ignore').read())]
    if dist:
        d = np.array(dist)
        print(f"\n--- strato reattivo: {len(d)} interventi, mediana {np.median(d):.2f} m ---")
        for lo, hi, lab in ((0, 0.45, 'soglia dura'), (0.45, 0.80, 'anticipato'),
                            (0.80, 9, 'ampio')):
            n = int(((d >= lo) & (d < hi)).sum())
            print(f"  {lo:.2f}-{hi:.2f} m ({lab}): {n:4d} ({100 * n / len(d):3.0f}%)")


if __name__ == '__main__':
    main()
