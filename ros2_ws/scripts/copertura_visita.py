#!/usr/bin/env python3
"""COPERTURA DI VISITA: quanta parte dello spazio percorribile il drone ha davvero visitato.

Non conta cio' che il lidar o la telecamera hanno visto da lontano, ma dove il
drone e' PASSATO. Dalla geometria vera del mondo (.sdf) si costruisce la pianta
alla quota di volo (box che intersecano 0.4-1.6 m) a 10 cm; lo spazio
percorribile sono le celle con almeno 'margine' m di distanza dagli ostacoli
raggiungibili dalla partenza. Una cella e' visitata se lungo il volo (posizione
VERA del simulatore, in aria) il drone e' passato entro 'raggio' m con la vista
libera (nessun ostacolo sulla congiungente).

Uso: copertura_visita.py volo.ulg [...] [--mondo cubicle_office] [--spawn=-13.5,-4]
     [--raggio 2.0] [--margine 0.5] [--png uscita.png]
"""
import argparse
import os
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import clearance_report as cr  # noqa: E402

RES = 0.1


def pianta(mondo_sdf):
    boxes = cr.boxes_from_sdf(mondo_sdf, 0.4, 1.6)
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    x0, x1 = min(xs) - 1, max(xs) + 1
    y0, y1 = min(ys) - 1, max(ys) + 1
    nx, ny = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
    occ = np.zeros((nx, ny), dtype=bool)
    for cx, cy, hx, hy in boxes:
        i0, i1 = max(0, int((cx - hx - x0) / RES)), min(nx - 1, int((cx + hx - x0) / RES))
        j0, j1 = max(0, int((cy - hy - y0) / RES)), min(ny - 1, int((cy + hy - y0) / RES))
        occ[i0:i1 + 1, j0:j1 + 1] = True
    return occ, x0, y0


def raggiungibili(libero, i0, j0):
    from collections import deque
    vis = np.zeros_like(libero)
    if not libero[i0, j0]:
        # partenza a ridosso di un ostacolo: si parte dalla cella libera piu' vicina
        ii, jj = np.nonzero(libero)
        k = np.argmin((ii - i0) ** 2 + (jj - j0) ** 2)
        i0, j0 = ii[k], jj[k]
    q = deque([(i0, j0)])
    vis[i0, j0] = True
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < libero.shape[0] and 0 <= b < libero.shape[1] and libero[a, b] and not vis[a, b]:
                vis[a, b] = True
                q.append((a, b))
    return vis


def visita(occ, x0, y0, percorso, raggio):
    """Celle entro 'raggio' da almeno un punto del percorso, con la vista libera."""
    nx, ny = occ.shape
    vis = np.zeros_like(occ)
    r = int(raggio / RES)
    off = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if di * di + dj * dj <= r * r]
    ultimo = None
    for x, y in percorso:
        if ultimo is not None and np.hypot(x - ultimo[0], y - ultimo[1]) < 0.15:
            continue
        ultimo = (x, y)
        ci, cj = int((x - x0) / RES), int((y - y0) / RES)
        for di, dj in off:
            a, b = ci + di, cj + dj
            if not (0 <= a < nx and 0 <= b < ny) or vis[a, b]:
                continue
            n = max(abs(di), abs(dj))
            ok = True
            for s in range(1, n):
                if occ[ci + round(di * s / n), cj + round(dj * s / n)]:
                    ok = False
                    break
            if ok:
                vis[a, b] = True
    return vis


def percorso_vero(ulg):
    from pyulog import ULog
    g = ULog(ulg, ['vehicle_local_position_groundtruth']).data_list[0].data
    x, y, z = g['y'], g['x'], -g['z']
    m = z > 0.5
    return list(zip(x[m], y[m]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('voli', nargs='+')
    ap.add_argument('--mondo', default='cubicle_office')
    ap.add_argument('--spawn', default='-13.5,-4')
    ap.add_argument('--raggio', type=float, default=2.0)
    ap.add_argument('--margine', type=float, default=0.5)
    ap.add_argument('--png', default='')
    a = ap.parse_args()
    sdf = os.path.join(QUI, '..', 'src', 'c2x_worlds', 'worlds', a.mondo + '.sdf')
    occ, x0, y0 = pianta(sdf)
    dist = cr.edt(occ) * RES
    sx, sy = (float(v) for v in a.spawn.split(','))
    area = raggiungibili(dist >= a.margine, int((sx - x0) / RES), int((sy - y0) / RES))
    for v in a.voli:
        perc = percorso_vero(v)
        vis = visita(occ, x0, y0, perc, a.raggio)
        quota = 100.0 * np.count_nonzero(vis & area) / max(1, np.count_nonzero(area))
        print(f'{v[-45:]:45s} spazio percorribile visitato (entro {a.raggio:.1f} m): {quota:5.1f}%  '
              f'mai visitato {np.count_nonzero(area & ~vis) * RES * RES:5.1f} m2 su {np.count_nonzero(area) * RES * RES:.0f}',
              flush=True)
        if a.png:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            img = np.ones(occ.shape + (3,))
            img[area] = (0.85, 0.85, 0.85)
            img[area & ~vis] = (1.0, 0.2, 0.2)
            img[occ] = (0.2, 0.2, 0.2)
            plt.figure(figsize=(9, 9))
            plt.imshow(np.transpose(img, (1, 0, 2)), origin='lower',
                       extent=(x0, x0 + occ.shape[0] * RES, y0, y0 + occ.shape[1] * RES))
            p = np.array(perc)
            plt.plot(p[:, 0], p[:, 1], 'b-', lw=0.5)
            plt.title(f'rosso = spazio percorribile mai visitato ({100 - quota:.1f}%), blu = volo')
            out = a.png if len(a.voli) == 1 else a.png.replace('.png', f'_{a.voli.index(v) + 1}.png')
            plt.savefig(out, dpi=80, bbox_inches='tight')
            plt.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
