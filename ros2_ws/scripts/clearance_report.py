#!/usr/bin/env python3
"""Misura il margine reale drone-ostacolo lungo un volo.

Perche' serve: i log del pianificatore campionano la posizione ~1 volta al
secondo, e a 0.35-0.6 m/s il drone si sposta fino a mezzo metro fra un campione
e l'altro. Dedurre da li' quanto e' passato vicino a un muro porta fuori strada
(e mi ci ha portato: avevo concluso "non stanno entrando nei muri" su quei dati,
mentre la verita' del simulatore mostra impulsi da contatto).

Qui si usano invece:
  - la posizione VERA del simulatore (vehicle_local_position_groundtruth), a
    ~100 Hz, non la stima dell'EKF;
  - la geometria di collisione EFFETTIVA, cioe' i box del mondo .sdf generato,
    non la nuvola di punti da cui derivano: la voxelizzazione li ingrossa, ed e'
    contro i box che il velivolo sbatte.

Uso: clearance_report.py <mondo.sdf> <spawn_x,spawn_y> <file.ulg> [...]
"""

import re
import sys

import numpy as np
from pyulog import ULog

RES = 0.05  # risoluzione della griglia di misura, in metri
# Raggio d'ingombro dell'x500 comprese le eliche: e' la soglia sotto la quale
# c'e' contatto fisico, non "poco margine".
DRONE_RADIUS = 0.35


def boxes_from_sdf(path, z_lo, z_hi):
    """Centri e semilati (x, y) dei box che intersecano la fascia di quota data.

    Il filtro in z e' essenziale: pavimento e soffitto coprono l'intera pianta,
    e proiettandoli in 2D senza filtrarli ogni cella risulterebbe occupata. Va
    anche letto un modello alla volta, altrimenti la posa di uno si accoppia con
    la dimensione del successivo.
    """
    text = open(path).read()
    out = []
    for mdl in re.finditer(r'<model name="[^"]*">(.*?)</model>', text, re.S):
        body = mdl.group(1)
        # Posa del modello: le pose delle singole forme sono relative a questa.
        mp = re.match(r'\s*<static>[^<]*</static>\s*<pose>([^<]+)</pose>', body, re.S)
        mpose = [float(v) for v in mp.group(1).split()] if mp else [0.0, 0.0, 0.0]
        # Un modello puo' contenere MOLTE forme di collisione, ognuna con la sua
        # posa: l'ambiente e' un unico modello statico con centinaia di box, non
        # un modello per box. Leggerne solo la prima misurerebbe il nulla.
        for col in re.finditer(
                r'<collision name="[^"]*">\s*<pose>([^<]+)</pose>\s*'
                r'<geometry><box><size>([^<]+)</size></box></geometry>', body, re.S):
            pose = [float(v) for v in col.group(1).split()]
            size = [float(v) for v in col.group(2).split()]
            if len(pose) < 3 or len(size) < 3:
                continue
            cz = mpose[2] + pose[2]
            if cz + size[2] / 2.0 < z_lo or cz - size[2] / 2.0 > z_hi:
                continue  # sopra o sotto la fascia di volo
            out.append((mpose[0] + pose[0], mpose[1] + pose[1], size[0] / 2.0, size[1] / 2.0))
    return out


def edt_1d(f):
    """Trasformata di distanza esatta 1D al quadrato (Felzenszwalb & Huttenlocher)."""
    n = len(f)
    d = np.empty(n)
    v = np.zeros(n, dtype=int)
    z = np.empty(n + 1)
    k = 0
    v[0] = 0
    z[0], z[1] = -np.inf, np.inf
    for q in range(1, n):
        while True:
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            if s <= z[k]:
                k -= 1
            else:
                break
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def edt(binary):
    """Distanza euclidea (in celle) dalla cella True piu' vicina."""
    big = 1e12
    f = np.where(binary, 0.0, big)
    for axis in (0, 1):
        f = np.apply_along_axis(edt_1d, axis, f)
    return np.sqrt(f)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    world, spawn_s, ulgs = sys.argv[1], sys.argv[2], sys.argv[3:]
    spawn = [float(v) for v in spawn_s.split(',')]

    # Fascia di quota di volo: e' li' che il velivolo puo' toccare qualcosa.
    boxes = boxes_from_sdf(world, 0.4, 1.6)
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    x0, x1 = min(xs) - 12, max(xs) + 12
    y0, y1 = min(ys) - 12, max(ys) + 12
    nx = int((x1 - x0) / RES) + 1
    ny = int((y1 - y0) / RES) + 1
    grid = np.zeros((nx, ny), dtype=bool)
    for cx, cy, hx, hy in boxes:
        i0 = max(0, int((cx - hx - x0) / RES))
        i1 = min(nx - 1, int((cx + hx - x0) / RES))
        j0 = max(0, int((cy - hy - y0) / RES))
        j1 = min(ny - 1, int((cy + hy - y0) / RES))
        grid[i0:i1 + 1, j0:j1 + 1] = True
    dist = edt(grid) * RES

    print(f'mondo: {len(boxes)} box, griglia {nx}x{ny} a {RES} m')
    print(f'{"log":>26} {"campioni":>9} {"minimo":>8} {"5%":>7} {"mediana":>8} {"sotto raggio":>13}')
    for path in ulgs:
        try:
            u = ULog(path, ['vehicle_local_position_groundtruth'])
            d = [x for x in u.data_list if 'groundtruth' in x.name][0]
        except Exception as exc:  # log troncato o topic assente
            print(f'{path.split("/")[-1]:>26}  non leggibile: {exc}')
            continue
        # Attenzione: vehicle_local_position (stima EKF) e' ancorato allo SPAWN,
        # ma vehicle_local_position_groundtruth e' gia' in coordinate MONDO
        # assolute. Sommare qui l'offset di spawn lo conterebbe due volte -
        # verificato sui range: gt.y sta attorno a -13.5, cioe' proprio la x di
        # spawn. Resta solo la conversione NED->ENU: x=Nord=+y mondo, y=Est=+x.
        wx = np.asarray(d.data['y'])
        wy = np.asarray(d.data['x'])
        z = -np.asarray(d.data['z'])
        air = z > 0.4  # solo volo, non decollo/atterraggio
        wx, wy = wx[air], wy[air]
        if wx.size == 0:
            print(f'{path.split("/")[-1]:>26}  nessun campione in volo')
            continue
        i = np.clip(((wx - x0) / RES).astype(int), 0, nx - 1)
        j = np.clip(((wy - y0) / RES).astype(int), 0, ny - 1)
        c = dist[i, j]
        below = 100.0 * (c < DRONE_RADIUS).mean()
        print(f'{path.split("/")[-1]:>26} {c.size:9d} {c.min():8.2f} '
              f'{np.percentile(c, 5):7.2f} {np.median(c):8.2f} {below:12.1f}%')
    return 0


if __name__ == '__main__':
    sys.exit(main())
