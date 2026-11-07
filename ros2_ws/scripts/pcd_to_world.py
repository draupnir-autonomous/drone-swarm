#!/usr/bin/env python3
"""Converte una mappa MARSIM (.pcd) in un mondo Gazebo (.sdf).

Le mappe di C2-Explorer sono nuvole di punti campionate sulle superfici e
consumate da MARSIM, che fa raycasting direttamente sui punti. Il nostro stack
usa Gazebo, che ha bisogno di geometria vera: senza superfici i raggi non
tornano, e un raggio senza ritorno non marca libero nulla (e' esattamente il
difetto che rendeva inosservabile meta' del vecchio mondo).

Le pareti di queste mappe sono verticali e a tutta altezza - verificato su
cubicle_office: il footprint a z=0.4, z=1.1 e z=2.0 coincide al 99.3% - quindi
una proiezione sul piano XY estrusa in altezza le ricostruisce fedelmente. Le
celle occupate vengono raggruppate in rettangoli massimali (greedy meshing) per
non generare decine di migliaia di box, che Gazebo non reggerebbe.
"""

import argparse
import sys

import numpy as np


def read_pcd(path):
    raw = open(path, 'rb').read()
    marker = b'DATA binary\n'
    idx = raw.find(marker)
    if idx < 0:
        raise SystemExit(f'{path}: atteso PCD binario')
    pts = np.frombuffer(raw[idx + len(marker):], dtype=np.float32).reshape(-1, 3)
    return pts


def footprint(pts, res, z_lo, z_hi):
    """Griglia 2D booleana delle celle attraversate da una superficie verticale."""
    sel = pts[(pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)]
    mins = pts[:, :2].min(0)
    ij = np.floor((sel[:, :2] - mins) / res).astype(int)
    nx = int(np.ceil((pts[:, 0].max() - mins[0]) / res)) + 1
    ny = int(np.ceil((pts[:, 1].max() - mins[1]) / res)) + 1
    grid = np.zeros((nx, ny), dtype=bool)
    ij = ij[(ij[:, 0] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] >= 0) & (ij[:, 1] < ny)]
    grid[ij[:, 0], ij[:, 1]] = True
    return grid, mins


def greedy_rects(grid):
    """Copre le celle occupate con rettangoli massimali, senza sovrapposizioni."""
    remaining = grid.copy()
    nx, ny = grid.shape
    rects = []
    for i in range(nx):
        j = 0
        while j < ny:
            if not remaining[i, j]:
                j += 1
                continue
            # estende in y finche' possibile
            j_end = j
            while j_end + 1 < ny and remaining[i, j_end + 1]:
                j_end += 1
            # poi estende in x finche' l'intera striscia resta libera
            i_end = i
            while i_end + 1 < nx and remaining[i_end + 1, j:j_end + 1].all():
                i_end += 1
            remaining[i:i_end + 1, j:j_end + 1] = False
            rects.append((i, j, i_end, j_end))
            j = j_end + 1
    return rects


# Una forma dentro il link condiviso, NON un modello a se'. Una versione
# precedente emetteva un <model> per rettangolo: 241 modelli, ognuno con posa,
# link canonico e aggiornamenti propri, per geometria che non si muove mai. E'
# spreco puro e appesantisce inutilmente il simulatore.
SHAPE = """      <collision name="c{name}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
      </collision>
      <visual name="v{name}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <transparency>{alpha}</transparency>
        <material>
          <ambient>{col}</ambient>
          <diffuse>{col}</diffuse>
        </material>
      </visual>
"""

MODEL_OPEN = """    <model name="{name}">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
"""

MODEL_CLOSE = """      </link>
    </model>
"""

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATO DA scripts/pcd_to_world.py - non modificare a mano.
     Sorgente: {src}
     Estensione: x [{x0:.2f}, {x1:.2f}]  y [{y0:.2f}, {y1:.2f}]  altezza {h:.2f} m
     {nrect} box di collisione da {ncell} celle occupate a {res} m. -->
<sdf version="1.9">
  <world name="{world}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Nessun blocco <plugin>, di proposito: dichiararne una lista esplicita
         SOSTITUISCE il set predefinito di Gazebo invece di aggiungersi. Una
         versione precedente di questo generatore elencava physics/sensors/imu/
         air-pressure e cosi' facendo lasciava fuori magnetometro e navsat: PX4
         rifiutava di armare con "Preflight Fail: Found 0 compass (required: 1)"
         e nessun drone decollava, pur continuando a pianificare a terra. Il
         mondo empty_room, che funziona, non dichiara plugin. -->

    <light type="directional" name="sunUTC">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.1 0.1 -0.9</direction>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('out')
    ap.add_argument('--world', default='cubicle_office')
    ap.add_argument('--res', type=float, default=0.2)
    ap.add_argument('--crop', type=float, default=0.0,
                    help='se >0, tiene solo i punti entro +-crop metri dal centro della '
                         'nuvola: serve a ottenere una scena piu' + chr(39) + ' piccola con la '
                         'stessa geometria reale (stanze, corsie, cubicoli) invece di una '
                         'scena finta. Il guscio perimetrale sigilla il taglio.')
    ap.add_argument('--ceiling-alpha', type=float, default=0.75,
                    help='trasparenza del solo soffitto: 0 = opaco, 1 = invisibile')
    ap.add_argument('--wall-height', type=float, default=None,
                    help='default: altezza del soffitto ricavata dalla nuvola')
    args = ap.parse_args()

    pts = read_pcd(args.pcd)
    if args.crop > 0.0:
        cx = 0.5 * (float(pts[:, 0].min()) + float(pts[:, 0].max()))
        cy = 0.5 * (float(pts[:, 1].min()) + float(pts[:, 1].max()))
        keep = ((np.abs(pts[:, 0] - cx) <= args.crop) & (np.abs(pts[:, 1] - cy) <= args.crop))
        pts = pts[keep]
        print(f'ritaglio +-{args.crop} m attorno a ({cx:.2f}, {cy:.2f}): '
              f'{keep.sum()} punti su {len(keep)}')
    z_min, z_max = float(pts[:, 2].min()), float(pts[:, 2].max())
    # Pavimento e soffitto vengono ricostruiti a parte: qui servono solo le
    # superfici verticali, quindi si scartano le fasce estreme in z.
    grid, mins = footprint(pts, args.res, z_min + 0.25, z_max - 0.15)
    rects = greedy_rects(grid)
    height = args.wall_height if args.wall_height else (z_max - 0.05)

    x0, y0 = float(mins[0]), float(mins[1])
    x1, y1 = float(pts[:, 0].max()), float(pts[:, 1].max())

    out = [HEADER.format(src=args.pcd, world=args.world, x0=x0, x1=x1, y0=y0, y1=y1,
                         h=height, nrect=len(rects), ncell=int(grid.sum()), res=args.res)]

    out.append(MODEL_OPEN.format(name='building'))

    for n, (i, j, ie, je) in enumerate(rects):
        sx = (ie - i + 1) * args.res
        sy = (je - j + 1) * args.res
        cx = x0 + (i * args.res) + sx / 2.0
        cy = y0 + (j * args.res) + sy / 2.0
        out.append(SHAPE.format(name=f'w{n}', cx=cx, cy=cy, cz=height / 2.0,
                                sx=sx, sy=sy, sz=height, col='0.55 0.55 0.6 1', alpha=0.0))

    # Guscio esterno: il perimetro ricostruito dalla nuvola puo' avere fessure di
    # una cella (in cubicle_office ne resta una all'angolo +x/+y, trovata con un
    # flood fill dall'esterno). Basta una fessura perche' dei raggi escano e non
    # tornino, e quella direzione resta inosservabile per sempre. Quattro pareti
    # sui bordi del bounding box garantiscono la tenuta a prescindere.
    shell = 0.2
    for n, (cx, cy, sx, sy) in enumerate([
            (x0 - shell / 2, (y0 + y1) / 2, shell, (y1 - y0) + 2 * shell),
            (x1 + shell / 2, (y0 + y1) / 2, shell, (y1 - y0) + 2 * shell),
            ((x0 + x1) / 2, y0 - shell / 2, (x1 - x0) + 2 * shell, shell),
            ((x0 + x1) / 2, y1 + shell / 2, (x1 - x0) + 2 * shell, shell)]):
        out.append(SHAPE.format(name=f's{n}', cx=cx, cy=cy, cz=height / 2.0,
                                sx=sx, sy=sy, sz=height, col='0.5 0.5 0.55 1', alpha=0.0))

    # Soffitto: chiude l'ambiente verso l'alto, cosi' i raggi diretti in su
    # tornano indietro e il volume da esplorare e' finito.
    # Soffitto reso trasparente SOLO nel visual, per poter guardare dentro
    # l'edificio dalla GUI. La collisione resta piena. Attenzione: il gpu_lidar
    # di Gazebo e' basato sul rendering, quindi legge la geometria VISIVA - se
    # la trasparenza lo rendesse invisibile ai raggi si ricreerebbe il difetto
    # dei raggi che escono e non tornano. Va verificato, non dato per scontato.
    out.append(SHAPE.format(name='ceil', cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0,
                            cz=height + 0.1, sx=(x1 - x0) + 0.4, sy=(y1 - y0) + 0.4,
                            sz=0.2, col='0.62 0.62 0.66 1', alpha=args.ceiling_alpha))
    out.append(MODEL_CLOSE)

    out.append('  </world>\n</sdf>\n')
    open(args.out, 'w').write(''.join(out))
    print(f'{args.out}: 1 modello statico con {len(rects) + 5} forme da {int(grid.sum())} celle, '
          f'estensione x[{x0:.2f},{x1:.2f}] y[{y0:.2f},{y1:.2f}] h={height:.2f}')


if __name__ == '__main__':
    sys.exit(main())
