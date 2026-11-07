#!/usr/bin/env python3
"""REPLAY 3D di un volo simulato (MP4 1920x1080, 60 fps), dai dati registrati.

La camera della GUI di Gazebo non si lascia guidare in modo fluido dall'esterno
(ogni spostamento richiesto e' un'animazione a se': la camera restava indietro di
6-9 m e procedeva a scatti). Qui la scena si ridisegna fuori schermo con Open3D a
partire dagli stessi dati: geometria dell'edificio dal mondo .sdf, modello X500 di
PX4 (telaio e motori Collada, LiDAR Mid-360 sul tetto), posizione e assetto VERI del
drone dal registro di volo di PX4 (vehicle_*_groundtruth). La camera virtuale segue
il drone con un movimento continuo (traiettoria della camera levigata nel tempo),
fondo nero, traiettoria arancione che si allunga, tempo e fase del volo in
sovrimpressione. Apertura dall'alto sull'intero ufficio e chiusura uguale.

Uso: video_replay3d.py <cartella_run> <indice> <uscita.mp4> [--mondo cubicle_office]
                       [--velocita 5] [--fps 60]
La cartella ha la forma delle campagne: rel_ulog_<i>_0.ulg, rel_expl_<i>.log.
"""
import argparse
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
from video_demo import font, mmss, codifica, rendi, ARANCIO, ARM, GRIGIO, W, H, RENDER_W, RENDER_H  # noqa: E402

MODELLI = os.path.join(QUI, '..', '..', 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'models')


# ------------------------------------------------------------ modello del drone
def carica_dae(path):
    """Triangoli di un file Collada (posizioni e indici, con le trasformazioni dei nodi)."""
    root = ET.parse(path).getroot()
    ns = root.tag.split('}')[0].strip('{')
    q = lambda t: f'{{{ns}}}{t}'  # noqa: E731
    geom = {}
    for g in root.iter(q('geometry')):
        mesh = g.find(q('mesh'))
        if mesh is None:
            continue
        src = {s.get('id'): s.find(q('float_array')) for s in mesh.findall(q('source'))}
        vert = mesh.find(q('vertices'))
        pid = [i.get('source')[1:] for i in vert.findall(q('input')) if i.get('semantic') == 'POSITION'][0]
        P = np.array(src[pid].text.split(), float).reshape(-1, 3)
        tris = []
        for tri in mesh.findall(q('triangles')):
            ins = tri.findall(q('input'))
            passo = max(int(i.get('offset')) for i in ins) + 1
            voff = [int(i.get('offset')) for i in ins if i.get('semantic') == 'VERTEX'][0]
            p = np.array(tri.find(q('p')).text.split(), int).reshape(-1, passo)[:, voff].reshape(-1, 3)
            tris.append(p)
        geom[g.get('id')] = (P, tris)
    out = []

    def visita(node, M):
        m = node.find(q('matrix'))
        if m is not None:
            M = M @ np.array(m.text.split(), float).reshape(4, 4)
        for ig in node.findall(q('instance_geometry')):
            P, tris = geom[ig.get('url')[1:]]
            Pw = (M[:3, :3] @ P.T).T + M[:3, 3]
            for t in tris:
                out.append((Pw, t))
        for ch in node.findall(q('node')):
            visita(ch, M)

    scena = root.find(q('library_visual_scenes')).find(q('visual_scene'))
    for n in scena.findall(q('node')):
        visita(n, np.eye(4))
    return out


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def modello_x500():
    """Mesh del drone nella terna del corpo (FLU), con colori; e dischi delle eliche."""
    import open3d as o3d
    base = os.path.join(MODELLI, 'x500_base', 'meshes')
    pezzi = []

    def aggiungi(dae, xyz, rpy, colore):
        for P, t in carica_dae(os.path.join(base, dae)):
            R = rot_rpy(*rpy)
            m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector((R @ P.T).T + np.array(xyz)),
                                          o3d.utility.Vector3iVector(t))
            m.paint_uniform_color(colore)
            pezzi.append(m)

    aggiungi('NXP-HGD-CF.dae', (0, 0, 0.025), (0, 0, math.pi), (0.13, 0.13, 0.14))
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        aggiungi('5010Base.dae', (0.174 * sx, 0.174 * sy, 0.032), (0, 0, -0.45), (0.55, 0.56, 0.6))
        aggiungi('5010Bell.dae', (0.174 * sx, 0.174 * sy, 0.06 - 0.032), (0, 0, 0), (0.75, 0.76, 0.8))
    # LiDAR Mid-360 sul tetto (x500_mid360: 0.25 m sopra base_link)
    lid = o3d.geometry.TriangleMesh.create_cylinder(radius=0.033, height=0.06, resolution=32)
    lid.translate((0, 0, 0.22))
    lid.paint_uniform_color((0.08, 0.08, 0.09))
    pezzi.append(lid)
    corpo = pezzi[0]
    for p in pezzi[1:]:
        corpo += p
    corpo.compute_vertex_normals()
    dischi = o3d.geometry.TriangleMesh()
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        d = o3d.geometry.TriangleMesh.create_cylinder(radius=0.145, height=0.004, resolution=48)
        d.translate((0.174 * sx, 0.174 * sy, 0.066))
        dischi += d
    dischi.compute_vertex_normals()
    return corpo, dischi


# ------------------------------------------------------------ ambiente
def edificio(mondo):
    """Box visibili dell'edificio (senza soffitto) e pavimento interno."""
    import open3d as o3d
    sdf = os.path.join(QUI, '..', 'src', 'c2x_worlds', 'worlds', mondo + '.sdf')
    t = open(sdf).read()
    corpo = re.search(r'<model name="building">(.*?)</model>', t, re.S).group(1)
    pareti = o3d.geometry.TriangleMesh()
    xs, ys = [], []
    for v in re.finditer(r'<visual name="([^"]*)">\s*<pose>([^<]+)</pose>\s*<geometry><box><size>([^<]+)</size>',
                         corpo):
        if 'ceil' in v.group(1):
            continue
        p = [float(a) for a in v.group(2).split()]
        s = [float(a) for a in v.group(3).split()]
        b = o3d.geometry.TriangleMesh.create_box(*s)
        b.translate((-s[0] / 2, -s[1] / 2, -s[2] / 2))
        b.rotate(rot_rpy(*(p[3:6] if len(p) >= 6 else (0, 0, 0))), center=(0, 0, 0))
        b.translate(p[:3])
        pareti += b
        xs += [p[0] - s[0] / 2, p[0] + s[0] / 2]
        ys += [p[1] - s[1] / 2, p[1] + s[1] / 2]
    pareti.compute_vertex_normals()
    pareti.paint_uniform_color((0.46, 0.47, 0.5))
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    pav = o3d.geometry.TriangleMesh.create_box(x1 - x0, y1 - y0, 0.02)
    pav.translate((x0, y0, -0.02))
    pav.compute_vertex_normals()
    pav.paint_uniform_color((0.66, 0.66, 0.68))
    return pareti, pav, (x0, x1, y0, y1)


# ------------------------------------------------------------ dati del volo
def volo(ulg):
    from pyulog import ULog
    u = ULog(ulg, ['vehicle_local_position_groundtruth', 'vehicle_attitude_groundtruth'])
    d = {m.name: m.data for m in u.data_list}
    g, a = d['vehicle_local_position_groundtruth'], d['vehicle_attitude_groundtruth']
    tg = g['timestamp'].astype(np.float64) / 1e6
    ta = a['timestamp'].astype(np.float64) / 1e6
    P = np.c_[g['y'], g['x'], -g['z']]
    Q = np.c_[a['q[0]'], a['q[1]'], a['q[2]'], a['q[3]']]
    return tg, P, ta, Q


def R_enu_flu(q):
    """Assetto: quaternione PX4 (corpo FRD -> NED) in matrice corpo FLU -> mondo ENU."""
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    B = np.diag([1, -1, -1])
    return A @ R @ B


def eventi(log):
    chiavi = {'via': r'giro iniziale concluso', 'fine_esplorazione': r'has finished exploration',
              'fine_visita': r'fase conclusa', 'appoggio': r'\[Casa\] sopra il punto di decollo'}
    out = {}
    for l in open(log, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\]', l)
        if m:
            for k, rx in chiavi.items():
                if k not in out and re.search(rx, l):
                    out[k] = float(m.group(1))
    return out


def offset_orologi(tg, P, log):
    """Scarto fra l'ora dei log ROS e il tempo del registro PX4 (posizioni [StuckDiag])."""
    T, Pl = [], []
    for l in open(log, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\].*StuckDiag\] pos\((\S+) (\S+) (\S+)\)', l)
        if m:
            T.append(float(m.group(1)))
            Pl.append((float(m.group(2)), float(m.group(3))))
    T, Pl = np.array(T), np.array(Pl)

    def err(o):
        return np.median(np.hypot(np.interp(T - o, tg, P[:, 0]) - Pl[:, 0], np.interp(T - o, tg, P[:, 1]) - Pl[:, 1]))
    grezzo = np.arange(T[0] - tg[-1], T[-1] - tg[0], 0.5)
    o = grezzo[np.argmin([err(v) for v in grezzo])]
    fine = np.arange(o - 1, o + 1, 0.02)
    return fine[np.argmin([err(v) for v in fine])]


def smoothstep(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def main():
    import open3d as o3d
    from PIL import Image, ImageDraw
    ap = argparse.ArgumentParser()
    ap.add_argument('cartella')
    ap.add_argument('run', type=int)
    ap.add_argument('uscita')
    ap.add_argument('--mondo', default='cubicle_office')
    ap.add_argument('--velocita', type=float, default=5.0)
    ap.add_argument('--fps', type=int, default=60)
    ap.add_argument('--offset', default='-7,-7,9.5', help='camera rispetto al drone (m, mondo)')
    ap.add_argument('--prova', type=float, default=-1.0, help='solo un fotogramma a questo tempo di missione (s)')
    ap.add_argument('--luce', default='NO_SHADOWS', help='profilo di luce di Open3D (le ombre fanno strisce)')
    ap.add_argument('--luce_sole', type=float, default=0.0)
    ap.add_argument('--luce_ambiente', type=float, default=0.0)
    a = ap.parse_args()
    fps, vel = a.fps, a.velocita
    tg, P, ta, Q = volo(os.path.join(a.cartella, f'rel_ulog_{a.run}_0.ulg'))
    log = os.path.join(a.cartella, f'rel_expl_{a.run}.log')
    off_log = offset_orologi(tg, P, log)
    ev = {k: v - off_log for k, v in eventi(log).items()}
    aria = np.nonzero(P[:, 2] > 0.3)[0]
    t_dec, t_app = tg[aria[0]], tg[aria[-1]]
    print(f'[3d] volo {t_app - t_dec:.0f} s', flush=True)
    # traiettoria della camera: punto inseguito levigato (gaussiana di 1.2 s, non causale)
    Tc = np.arange(t_dec - 2, t_app + 2, 0.05)
    Pc = np.c_[[np.interp(Tc, tg, P[:, k]) for k in range(3)]].T
    sig = int(1.2 / 0.05)
    ker = np.exp(-0.5 * (np.arange(-3 * sig, 3 * sig + 1) / sig) ** 2)
    ker /= ker.sum()
    Pl = np.c_[[np.convolve(np.pad(Pc[:, k], 3 * sig, mode='edge'), ker, mode='valid') for k in range(3)]].T
    off = np.array([float(v) for v in a.offset.split(',')])

    r = o3d.visualization.rendering.OffscreenRenderer(RENDER_W, RENDER_H)  # Jetson: vedi video_demo.py
    sc = r.scene
    sc.set_background([0, 0, 0, 1])
    profilo = getattr(o3d.visualization.rendering.Open3DScene.LightingProfile, a.luce)
    sc.set_lighting(profilo, np.array([0.4, 0.6, -0.7]))
    if a.luce_sole > 0:
        sc.scene.set_sun_light(np.array([0.35, 0.55, -0.75]), np.array([1.0, 0.98, 0.95]), a.luce_sole)
        sc.scene.enable_sun_light(True)
    if a.luce_ambiente > 0:
        sc.scene.set_indirect_light_intensity(a.luce_ambiente)
    lit = o3d.visualization.rendering.MaterialRecord()
    lit.shader = 'defaultLit'
    pareti, pav, (x0, x1, y0, y1) = edificio(a.mondo)
    sc.add_geometry('pareti', pareti, lit)
    sc.add_geometry('pavimento', pav, lit)
    corpo, dischi = modello_x500()
    if ARM:
        # Jetson: col modello intero (210k triangoli) il ~5% dei fotogrammi usciva
        # tutto nero, sempre nelle stesse inquadrature; a 40k triangoli 0 su 300 (il
        # drone occupa pochi pixel, la differenza non si vede).
        corpo = corpo.simplify_quadric_decimation(40000)
        corpo.compute_vertex_normals()
    sc.add_geometry('drone', corpo, lit)
    trasp = o3d.visualization.rendering.MaterialRecord()
    trasp.shader = 'defaultLitTransparency'
    trasp.base_color = [0.1, 0.1, 0.12, 0.35]
    sc.add_geometry('eliche', dischi, trasp)
    unlit = o3d.visualization.rendering.MaterialRecord()
    unlit.shader = 'unlitLine'
    unlit.line_width = 5.0
    centro = np.array([(x0 + x1) / 2, (y0 + y1) / 2, 0.0])
    alto = centro + np.array([-0.45 * (x1 - x0), -0.75 * (y1 - y0), 0.95 * (x1 - x0)])

    def drone_a(t):
        p = np.array([np.interp(t, tg, P[:, k]) for k in range(3)])
        i = min(max(int(np.searchsorted(ta, t)), 1), len(ta) - 1)
        u = (t - ta[i - 1]) / max(1e-6, ta[i] - ta[i - 1])
        q = Q[i - 1] * (1 - u) + Q[i] * u
        q /= np.linalg.norm(q)
        T = np.eye(4)
        T[:3, :3] = R_enu_flu(q)
        T[:3, 3] = p
        return T

    def camera_a(t, tv, tv_fine):
        mira = np.array([np.interp(t, Tc, Pl[:, k]) for k in range(3)])
        occhio = mira + off
        # apertura e chiusura dall'alto sull'intero ufficio
        u = smoothstep((tv - 1.5) / 3.0) * (1 - smoothstep((tv - (tv_fine - 4.5)) / 3.0))
        return centro * (1 - u) + mira * u, alto * (1 - u) + occhio * u

    f_tit, f_sub, f_lab, f_num, f_chip = font(30, True), font(19), font(15, True), font(34, True), font(15, True)
    fasi = [('TAKE-OFF AND INITIAL SCAN', (120, 140, 170)), ('EXPLORATION', (66, 133, 244)),
            ('FULL VISIT', (22, 104, 98)), ('RETURN AND LANDING', ARANCIO), ('LANDED', (60, 160, 90))]

    def fase(t):
        if t < ev.get('via', t_dec + 15):
            return fasi[0]
        if t < ev.get('fine_esplorazione', 1e18):
            return fasi[1]
        if 'fine_visita' in ev and t < ev['fine_visita']:
            return fasi[2]
        if t < t_app + 0.5:
            return fasi[3]
        return fasi[4]

    t_inizio, t_fine = t_dec - 1.0, t_app + 2.0
    n_volo = int((t_fine - t_inizio) / vel * fps)
    n_coda = int(3.0 * fps)                      # chiusura dall'alto a drone atterrato
    totale = n_volo + n_coda
    tv_fine = totale / fps
    cartella = os.path.splitext(a.uscita)[0] + '_frame'
    if a.prova < 0:
        shutil.rmtree(cartella, ignore_errors=True)
        os.makedirs(cartella)
    Ttraj = np.arange(t_dec, t_app, 0.1)
    Ptraj = np.c_[[np.interp(Ttraj, tg, P[:, k]) for k in range(3)]].T
    PEZZO = 100        # punti per pezzo fisso della traiettoria (10 s di volo)
    n_pezzi = 0

    def linea(pts):
        ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts),
                                  o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(pts) - 1)]))
        ls.paint_uniform_color((1.0, 0.33, 0.0))
        return ls
    indici = [int((t_dec + a.prova - t_inizio) / vel * fps)] if a.prova >= 0 else range(totale)
    for n in indici:
        tv = n / fps
        t = min(t_inizio + tv * vel, t_fine)
        T = drone_a(t)
        sc.set_geometry_transform('drone', T)
        sc.set_geometry_transform('eliche', T)
        # traiettoria fino a t. I tratti gia' percorsi si aggiungono una volta sola, a
        # pezzi da PEZZO punti; a ogni fotogramma si ricarica solo la coda. Ricaricarla
        # tutta ogni volta, sul Jetson, faceva uscire neri sempre piu' fotogrammi man
        # mano che la traiettoria si allungava (volo di 16 min: fino al 69% dei
        # fotogrammi in un decimo del video), e il video andava a scatti.
        k = int(np.searchsorted(Ttraj, t))
        if k >= 2:
            while k - 1 >= (n_pezzi + 1) * PEZZO:
                sc.add_geometry(f'traiettoria_{n_pezzi}',
                                linea(Ptraj[n_pezzi * PEZZO:(n_pezzi + 1) * PEZZO + 1]), unlit)
                n_pezzi += 1
            if sc.has_geometry('traiettoria'):
                sc.remove_geometry('traiettoria')
            sc.add_geometry('traiettoria', linea(np.vstack([Ptraj[n_pezzi * PEZZO:k], T[:3, 3]])), unlit)
        mira, occhio = camera_a(t, tv, tv_fine)
        r.setup_camera(45.0, mira, occhio, [0, 0, 1])
        im = rendi(r)
        dr = ImageDraw.Draw(im, 'RGBA')
        dr.rectangle([0, 0, W, 92], fill=(0, 0, 0, 200))
        dr.text((32, 14), 'Autonomous flight: 3D replay', font=f_tit, fill=(245, 248, 252))
        dr.text((32, 56), f'SIMULATION  ·  Gazebo + PX4  ·  true position and attitude of the simulated drone  ·  '
                          f'{vel:g}x speed', font=f_sub, fill=GRIGIO)
        nome, col = fase(t)
        dr.rounded_rectangle([24, H - 112, 600, H - 24], radius=14, fill=(0, 0, 0, 200))
        dr.text((44, H - 100), 'MISSION TIME', font=f_lab, fill=GRIGIO)
        dr.text((44, H - 78), mmss(t - t_dec), font=f_num, fill=(245, 248, 252))
        w = dr.textlength(nome, font=f_chip) + 28
        dr.rounded_rectangle([250, H - 84, 250 + w, H - 50], radius=16, fill=col)
        dr.text((264, H - 76), nome, font=f_chip, fill=(255, 255, 255))
        if a.prova >= 0:
            im.save(a.uscita)
            return 0
        im.save(os.path.join(cartella, f'f{n:05d}.jpg'), quality=93)
        if n % 600 == 0:
            print(f'[3d]   {n}/{totale}', flush=True)
    print(f'[3d] codifica di {totale} fotogrammi ({totale / fps:.0f} s)...', flush=True)
    codifica(cartella, a.uscita, fps)
    shutil.rmtree(cartella, ignore_errors=True)
    print(f'[3d] pronto: {a.uscita}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
