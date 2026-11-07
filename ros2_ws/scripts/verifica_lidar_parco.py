#!/usr/bin/env python3
"""Il lidar simulato vede tutto cio' con cui il drone puo' urtare, nel mondo "parco"?

Mette nel parco dei lidar fermi identici a quello del drone (300 x 64, +-50 gradi,
senza rumore) in punti liberi a quota di volo, prende una scansione da ciascuno e la
confronta RAGGIO PER RAGGIO con un raycast sulla geometria di COLLISIONE dei modelli:
per ogni pezzo del mondo, dei raggi che la collisione dice fermati li' entro --portata
metri, quanti il lidar vede davvero (distanza misurata entro 10 cm da quella attesa),
separando le facce frontali da quelle posteriori dei triangoli.

Perche' (18 ottobre 2026): il lidar di Gazebo rispetta la trasparenza delle texture e
scarta le facce posteriori anche con double_sided, la collisione no. Prima della
correzione (make_parco.geometrie_per_lidar) le chiome erano viste al 15-20% e la
facciata della casa a sud-ovest all'1%; il drone ha urtato un ramo e sfiorato quella
facciata. Da rifare dopo aver cambiato modelli o versione di Gazebo.

Uso:  python3 scripts/verifica_lidar_parco.py [--rig 40] [--portata 8]
Non va lanciato insieme a una simulazione: usa la sua GZ_PARTITION, ma la CPU e' una.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import make_parco as mp  # noqa: E402
from modelli_geo import pose_matrix, risolvi_uri, triangoli_dae, triangoli_obj  # noqa: E402

ROS_SETUP = '/opt/ros/jazzy/setup.bash'


def scatola(cx, cy, cz, sx, sy, sz):
    V = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2)
                  for z in (-sz / 2, sz / 2)]) + [cx, cy, cz]
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def collisioni():
    """(nome, V, F) dei triangoli di collisione in coordinate mondo: modelli, siepi,
    marciapiede (il terreno lo aggiunge il chiamante)."""
    out = []
    for nome, cartella, x, y, yaw in mp.oggetti():
        d = os.path.join(mp.MODELLI, cartella)
        m = ET.fromstring(open(os.path.join(d, 'model.sdf')).read()).find('model')
        Mm = pose_matrix(f'{x} {y} 0 0 0 {yaw}') @ pose_matrix(m.findtext('pose'))
        for link in m.findall('link'):
            Ml = Mm @ pose_matrix(link.findtext('pose'))
            for col in link.findall('collision'):
                Mc = Ml @ pose_matrix(col.findtext('pose'))
                mesh = col.find('geometry/mesh')
                f = risolvi_uri(mesh.findtext('uri'), d, [mp.MODELLI])
                V, F = triangoli_dae(f) if f.lower().endswith('.dae') else triangoli_obj(f)
                if mesh.findtext('scale'):
                    V = V * np.array([float(v) for v in mesh.findtext('scale').split()])
                out.append((nome, (Mc @ np.c_[V, np.ones(len(V))].T).T[:, :3], F))
    for nome, cx, cy, cz, sx, sy, sz, _ in mp.siepi():
        out.append((nome, *scatola(cx, cy, cz, sx, sy, sz)))
    m0, m1 = mp.MARCIAPIEDE
    out.append(('marciapiede', *scatola(0.0, (m0 + m1) / 2, 0.06, 2 * mp.MEZZO + 2 * mp.SIEPE_S + 12.0,
                                        m1 - m0, 0.12)))
    return out


def scena_collisioni():
    import open3d as o3d
    scena = o3d.t.geometry.RaycastingScene()
    etich = {}
    for nome, V, F in collisioni():
        etich[scena.add_triangles(o3d.core.Tensor(V.astype(np.float32)),
                                  o3d.core.Tensor(F.astype(np.uint32)))] = nome
    V = np.array([[-100, -100, 0], [100, -100, 0], [100, 100, 0], [-100, 100, 0]], np.float32)
    etich[scena.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(np.array([[0, 1, 2], [0, 2, 3]],
                                                                         np.uint32)))] = 'terreno'
    return scena, etich


def scrivi_mondo(out, n_rig, scena):
    import open3d as o3d
    rng = np.random.default_rng(7)
    rig = []
    while len(rig) < n_rig:
        p = np.array([rng.uniform(-15, 15), rng.uniform(-15.2, 15), [0.9, 1.5][len(rig) % 2]], np.float32)
        if scena.compute_distance(o3d.core.Tensor(p[None])).numpy()[0] > 0.7:
            rig.append([float(v) for v in p])
    s = open(os.path.join(mp.MONDI, 'parco.sdf')).read()
    plug = ('<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>\n'
            '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>\n'
            '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
            '<render_engine>ogre2</render_engine></plugin>\n')
    lidar = ''.join(f'''<model name="rig{k}"><static>true</static><pose>{x:.3f} {y:.3f} {z} 0 0 0</pose><link name="l">
<sensor name="lidar" type="gpu_lidar"><topic>/verifica/lidar{k}</topic><update_rate>1</update_rate><always_on>1</always_on>
<ray><scan><horizontal><samples>300</samples><resolution>1</resolution><min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle></horizontal>
<vertical><samples>64</samples><resolution>1</resolution><min_angle>-0.8726646259971648</min_angle><max_angle>0.8726646259971648</max_angle></vertical></scan>
<range><min>0.1</min><max>55.0</max><resolution>0.01</resolution></range></ray></sensor></link></model>
''' for k, (x, y, z) in enumerate(rig))
    i = s.index('<world name="parco">') + len('<world name="parco">')
    s = s[:i] + '\n' + plug + s[i:]
    open(out, 'w').write(s.replace('  </world>', lidar + '  </world>'))
    return rig


def cattura(n, attesa):
    from gz.transport13 import Node
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    dati = {}

    def fa(k):
        def cb(m):
            if k in dati:
                return
            f = {x.name: x.offset for x in m.field}
            buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            xyz = np.stack([buf[:, f[c]:f[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], 1)
            dati[k] = xyz.reshape(m.height, m.width, 3)
        return cb
    node = Node()
    for k in range(n):
        node.subscribe(PointCloudPacked, f'/verifica/lidar{k}/points', fa(k))
    t0 = time.time()
    while len(dati) < n and time.time() - t0 < attesa:
        time.sleep(0.5)
    return dati


def confronta(rig, dati, scena, etich, portata):
    import open3d as o3d
    tot = {}
    estranei = [0, 0]
    for k, (rx, ry, rz) in enumerate(rig):
        if k not in dati:
            continue
        A = dati[k]
        H, W = A.shape[:2]
        P = A.reshape(-1, 3).astype(np.float64)
        fin = np.isfinite(P).all(1)
        # Direzione di ogni raggio, compresi quelli senza ritorno (+inf): azimut per
        # colonna ed elevazione per riga, stimati dai raggi che sono tornati.
        az = np.arctan2(P[:, 1], P[:, 0])
        el = np.arctan2(P[:, 2], np.hypot(P[:, 0], P[:, 1]))
        riga = np.repeat(np.arange(H), W)
        col = np.tile(np.arange(W), H)
        azc = np.array([np.angle(np.mean(np.exp(1j * az[fin & (col == j)])))
                        if (fin & (col == j)).any() else np.nan for j in range(W)])
        elr = np.array([np.median(el[fin & (riga == i)]) if (fin & (riga == i)).any() else np.nan
                        for i in range(H)])
        j = np.arange(W); ok = ~np.isnan(azc)
        if not ok.all():
            azc[~ok] = np.interp(j[~ok], j[ok], np.unwrap(azc[ok]))
        i = np.arange(H); ok = ~np.isnan(elr)
        if not ok.all():
            elr[~ok] = np.polyval(np.polyfit(i[ok], elr[ok], 1), i[~ok])
        a, e = azc[col], elr[riga]
        D = np.c_[np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)]
        rmis = np.where(fin, np.linalg.norm(P, axis=1), np.inf)
        ans = scena.cast_rays(o3d.core.Tensor(np.c_[np.tile([rx, ry, rz], (len(D), 1)), D].astype(np.float32)))
        rexp = ans['t_hit'].numpy().astype(np.float64)
        gid = ans['geometry_ids'].numpy()
        fronte = (ans['primitive_normals'].numpy() * D).sum(1) < 0
        for g in np.unique(gid[np.isfinite(rexp)]):
            nome = etich.get(int(g), '?')
            if nome == 'terreno':
                continue
            m = (gid == g) & (rexp < portata)
            for lato, mm in (('F', m & fronte), ('R', m & ~fronte)):
                visti = np.abs(rmis[mm] - rexp[mm]) < 0.10
                t = tot.setdefault((nome, lato), np.zeros(2))
                t += [mm.sum(), visti.sum()]
        zm = rz + np.where(fin, P[:, 2], 0)
        estranei[0] += (fin & (rmis < portata) & (zm > 0.15) & (rmis < rexp - 0.10)).sum()
        estranei[1] += (fin & (rmis < portata)).sum()
    return tot, estranei


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rig', type=int, default=40, help='quanti lidar fermi (default 40)')
    ap.add_argument('--portata', type=float, default=8.0, help='raggi considerati, in metri (default 8)')
    ap.add_argument('--attesa', type=float, default=300.0, help='secondi massimi per le scansioni')
    a = ap.parse_args()
    scena, etich = scena_collisioni()
    cartella = tempfile.mkdtemp(prefix='c2x_verifica_lidar_')
    mondo = os.path.join(cartella, 'parco_verifica.sdf')
    rig = scrivi_mondo(mondo, a.rig, scena)
    env = dict(os.environ, GZ_PARTITION='c2x_verifica_lidar', GZ_SIM_RESOURCE_PATH=mp.MODELLI)
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']
    log = open(os.path.join(cartella, 'gz.log'), 'w')
    gz = subprocess.Popen(['bash', '-c', f'source {ROS_SETUP} && exec gz sim -r -s "{mondo}"'],
                          env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(20)
        dati = cattura(len(rig), a.attesa)
    finally:
        os.killpg(gz.pid, signal.SIGTERM)
        try:
            gz.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(gz.pid, signal.SIGKILL)
    print(f'scansioni: {len(dati)} di {len(rig)} (mondo e log in {cartella})')
    tot, estranei = confronta(rig, dati, scena, etich, a.portata)
    print(f'{"pezzo":18s} {"raggi F":>8s} {"visti F":>8s} {"raggi R":>8s} {"visti R":>8s} {"visti":>6s}')
    for n in sorted({n for n, _ in tot}):
        f = tot.get((n, 'F'), np.zeros(2)); r = tot.get((n, 'R'), np.zeros(2))
        pf = f'{100 * f[1] / f[0]:.0f}%' if f[0] else '-'
        pr = f'{100 * r[1] / r[0]:.0f}%' if r[0] else '-'
        print(f'{n:18s} {int(f[0]):8d} {pf:>8s} {int(r[0]):8d} {pr:>8s} '
              f'{100 * (f[1] + r[1]) / max(f[0] + r[0], 1):5.0f}%')
    print('F/R = facce frontali/posteriori dei triangoli di collisione rispetto al lidar.')
    print(f'ritorni dove la collisione non c\'e\': {estranei[0]} su {estranei[1]} entro {a.portata:.0f} m')
    json.dump({'rig': rig}, open(os.path.join(cartella, 'rig.json'), 'w'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
