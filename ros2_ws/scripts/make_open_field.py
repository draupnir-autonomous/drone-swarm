#!/usr/bin/env python3
"""Genera un mondo Gazebo "campo aperto con ostacoli".

Perche' recintato e non completamente aperto: in campo totalmente libero i raggi
lidar orizzontali non incontrano nulla e non tornano, e un raggio senza ritorno
non marca libero niente. Quel volume resterebbe sconosciuto per sempre e
l'esplorazione a frontiere non potrebbe concludersi - e' lo stesso difetto che
rendeva impossibile la convergenza nella stanza senza soffitto.
Il confine perimetrale restituisce i raggi in orizzontale; verso l'alto invece il
mondo resta APERTO (nessun soffitto), ed e' li' che entra in gioco il soffitto
virtuale della mappa: limita la quota di volo, mentre a delimitare l'esplorazione
e' box_max_z.

Il perimetro e' alto piu' della box di esplorazione di proposito, cosi' anche le
celle piu' alte da esplorare sono raggiungibili da raggi quasi orizzontali che
terminano sul confine.

Uso: make_open_field.py <out.sdf> [--half 20] [--fence-height 5]
"""

import argparse
import math
import random

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATO DA scripts/make_open_field.py - non modificare a mano. -->
<sdf version="1.9">
  <world name="open_field">
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
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Nessun blocco <plugin>: una lista esplicita SOSTITUISCE il set
         predefinito di Gazebo, e omettendo magnetometro e navsat PX4 rifiuta di
         armare ("Preflight Fail: Found 0 compass"). -->

    <light type="directional" name="sunUTC">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.95 0.95 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.3 0.2 -0.9</direction>
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
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.45 0.5 0.35 1</ambient>
            <diffuse>0.45 0.5 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""

BOX = """      <collision name="c{n}">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 0 0</pose>
        <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
      </collision>
      <visual name="v{n}">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 0 0</pose>
        <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
        <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>
      </visual>
"""

CYL = """      <collision name="c{n}">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 0 0</pose>
        <geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry>
      </collision>
      <visual name="v{n}">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 0 0</pose>
        <geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry>
        <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>
      </visual>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--half', type=float, default=20.0, help='semilato del campo, in metri')
    ap.add_argument('--fence-height', type=float, default=5.0)
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    rnd = random.Random(args.seed)
    h = args.half
    out = [HEADER]

    # Tutto in UN solo modello statico: un <model> per ostacolo costa a Gazebo
    # una posa e un link canonico ciascuno, per geometria che non si muove mai.
    out.append('    <model name="field">\n      <static>true</static>\n'
               '      <pose>0 0 0 0 0 0</pose>\n      <link name="link">\n')

    n = 0
    fh = args.fence_height
    for cx, cy, sx, sy in ((h, 0, 0.2, 2 * h + 0.2), (-h, 0, 0.2, 2 * h + 0.2),
                           (0, h, 2 * h + 0.2, 0.2), (0, -h, 2 * h + 0.2, 0.2)):
        out.append(BOX.format(n=f'fence{n}', cx=cx, cy=cy, cz=fh / 2, sx=sx, sy=sy, sz=fh,
                              col='0.40 0.42 0.38 1'))
        n += 1

    # Ostacoli sparsi: alberi (cilindri) e container (parallelepipedi), di altezze
    # diverse, con separazione minima cosi' restano corridoi percorribili.
    placed = []

    def far_enough(x, y, rad):
        if math.hypot(x, y) < 6.0:  # area di decollo libera
            return False
        for px, py, pr in placed:
            if math.hypot(x - px, y - py) < rad + pr + 3.0:
                return False
        return True

    tries = 0
    while len(placed) < 26 and tries < 4000:
        tries += 1
        x = rnd.uniform(-h + 3, h - 3)
        y = rnd.uniform(-h + 3, h - 3)
        if rnd.random() < 0.55:
            r = rnd.uniform(0.35, 0.8)
            if not far_enough(x, y, r):
                continue
            hh = rnd.uniform(2.0, 4.5)
            out.append(CYL.format(n=f'tree{len(placed)}', cx=x, cy=y, cz=hh / 2, r=r, h=hh,
                                  col='0.35 0.28 0.20 1'))
            placed.append((x, y, r))
        else:
            sx, sy = rnd.uniform(1.5, 4.0), rnd.uniform(1.5, 4.0)
            rad = max(sx, sy) / 2
            if not far_enough(x, y, rad):
                continue
            hh = rnd.uniform(1.2, 3.5)
            out.append(BOX.format(n=f'box{len(placed)}', cx=x, cy=y, cz=hh / 2, sx=sx, sy=sy,
                                  sz=hh, col='0.55 0.50 0.45 1'))
            placed.append((x, y, rad))

    out.append('      </link>\n    </model>\n')
    out.append('  </world>\n</sdf>\n')
    open(args.out, 'w').write(''.join(out))
    print(f'{args.out}: campo {2*h:.0f}x{2*h:.0f} m, recinzione alta {fh:.1f} m, '
          f'{len(placed)} ostacoli, nessun soffitto')


if __name__ == '__main__':
    main()
