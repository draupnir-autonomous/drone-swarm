#!/usr/bin/env python3
"""Misura del dondolio avanti-indietro da un log PX4 (.ulg) o da una cache .npz.

  inversioni/min  il verso del moto orizzontale si capovolge (velocita' a 1 s di
                  distanza con coseno < -0.3, entrambe > 0.3 m/s), conteggi
                  distanti almeno 1.5 s
  episodi         finestre di 8 s con >= 2 m percorsi e spostamento netto < 25%
                  del percorso (il drone va e torna), fuse se si sovrappongono
  rif_dietro%     quota del tempo in moto (> 0.3 m/s) con il riferimento piu' di
                  0.3 m ALLE SPALLE del drone lungo il moto (stima PX4 e
                  riferimento nella stessa terna locale)

Uso: dondolio.py volo1.ulg [volo2.ulg ...]   (anche cache .npz)
"""
import sys
import numpy as np


def carica(p):
    if p.endswith('.npz'):
        c = np.load(p)
        # cache: riferimento gia' riportato in coordinate mondo, confrontato con la verita'
        return (c['tg'], c['gx'], c['gy'], c['gz'], c['vx'], c['vy'], c['ts'], c['sx'], c['sy'],
                c['tg'], c['gx'], c['gy'])
    from pyulog import ULog
    u = ULog(p, ['vehicle_local_position_groundtruth', 'trajectory_setpoint', 'vehicle_local_position'])
    d = {m.name: m.data for m in u.data_list}
    g = d['vehicle_local_position_groundtruth']; s = d['trajectory_setpoint']; e = d['vehicle_local_position']
    # Tempi in float PRIMA di sottrarre: sono uint64, e un campione precedente
    # a t0 andava a capo (2^64) rendendo i tempi non monotoni.
    t0 = float(g['timestamp'][0])
    tt = lambda m: (m['timestamp'].astype(np.float64) - t0) / 1e6
    return (tt(g), g['y'], g['x'], -g['z'], g['vy'], g['vx'],
            tt(s), s['position[1]'], s['position[0]'], tt(e), e['y'], e['x'])


def misura(p):
    tg, gx, gy, gz, vx, vy, ts, sx, sy, te, ex, ey = carica(p)
    aria = np.nonzero(gz - gz[0] > 0.6)[0]
    if len(aria) == 0:
        return None
    ta, tb = tg[aria[0]] + 20, tg[aria[-1]] - 5
    T = np.arange(ta, tb, 0.1)
    X, Y = np.interp(T, tg, gx), np.interp(T, tg, gy)
    VX, VY = np.interp(T, tg, vx), np.interp(T, tg, vy)
    sp = np.hypot(VX, VY)
    inv, ultimo = 0, -99
    for k in range(10, len(T)):
        if sp[k] > 0.3 and sp[k - 10] > 0.3:
            c = (VX[k] * VX[k - 10] + VY[k] * VY[k - 10]) / (sp[k] * sp[k - 10])
            if c < -0.3 and T[k] - ultimo > 1.5:
                inv += 1
                ultimo = T[k]
    W = 80
    ep, fine = 0, -99
    for k in range(0, len(T) - W, 5):
        seg = np.c_[X[k:k + W + 1:5], Y[k:k + W + 1:5]]
        strada = np.linalg.norm(np.diff(seg, axis=0), axis=1).sum()
        netto = np.linalg.norm(seg[-1] - seg[0])
        if strada > 2.0 and netto < 0.25 * strada:
            if T[k] > fine:
                ep += 1
            fine = T[k] + 8
    m = (ts > ta) & (ts < tb)
    TS = ts[m]
    rx, ry = sx[m] - np.interp(TS, te, ex), sy[m] - np.interp(TS, te, ey)
    dvx, dvy = np.interp(TS, tg, vx), np.interp(TS, tg, vy)
    vn = np.hypot(dvx, dvy)
    moto = vn > 0.3
    lungo = (rx * dvx + ry * dvy) / np.maximum(vn, 1e-6)
    indietro = 100.0 * np.count_nonzero(moto & (lungo < -0.3)) / max(1, np.count_nonzero(moto))
    minuti = (tb - ta) / 60
    return dict(minuti=minuti, inv=inv / minuti, ep=ep, ind=indietro, vmed=sp.mean())


if __name__ == '__main__':
    print(f'{"volo":40s} {"min":>5} {"inv/min":>7} {"episodi":>7} {"rif_dietro%":>11} {"v.med":>5}')
    for p in sys.argv[1:]:
        r = misura(p)
        if r:
            print(f'{p[-40:]:40s} {r["minuti"]:5.1f} {r["inv"]:7.1f} {r["ep"]:7d} {r["ind"]:11.1f} {r["vmed"]:5.2f}',
                  flush=True)
