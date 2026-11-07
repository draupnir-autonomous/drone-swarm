#!/usr/bin/env python3
"""Pagella di una campagna a drone singolo: una riga per run, poi il riepilogo.

Mette insieme in un colpo solo le misure che servono per dire se il volo e'
ROBUSTO, non solo se "e' finito":

  esito      esplorazione conclusa / tetto di tempo / crash / stallo
  frontiere  quante ne restavano alla conclusione (0 = esplorazione completa)
  rientro    se il drone e' tornato alla partenza ed e' atterrato
  margine    distanza minima VERA dagli ostacoli (geometria del mondo .sdf
             contro la posizione vera del simulatore, come clearance_report.py)
  quota      quota minima vera in crociera
  incl.      inclinazione massima (sopra ~35 gradi il volo e' gia' al limite)
  vel.       velocita' orizzontale massima vera
  err.stima  errore della posizione stimata da PX4 rispetto alla verita'

Uso: analisi_campagna.py [cartella_campagna] [--mondo office_small] [--spawn -3.5,-3]
     (default: /tmp/c2x_reliability, office_small, prima posa di office_small)
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import clearance_report as cr  # noqa: E402  (stessa misura del margine, non una copia)

try:
    from pyulog import ULog
except ImportError:
    ULog = None


def conta(testo, motivo):
    return len(re.findall(motivo, testo))


def griglia_distanze(mondo_sdf):
    # Mondi fatti di modelli con mesh (parco): la pianta vera degli ostacoli nella
    # fascia di volo la prepara il generatore del mondo (make_parco.py) accanto al
    # .sdf, perche' qui si leggono solo i box.
    pianta = os.path.splitext(os.path.realpath(mondo_sdf))[0] + '_ostacoli.npz'
    if os.path.exists(pianta):
        z = np.load(pianta)
        if abs(float(z['res']) - cr.RES) < 1e-9:
            d = z['dist']
            return d, float(z['x0']), float(z['y0']), d.shape[0], d.shape[1]
    boxes = cr.boxes_from_sdf(mondo_sdf, 0.4, 1.6)
    if not boxes:
        return None   # mondo senza box: margine solo in 3D (distanze_3d)
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    x0, x1 = min(xs) - 12, max(xs) + 12
    y0, y1 = min(ys) - 12, max(ys) + 12
    nx = int((x1 - x0) / cr.RES) + 1
    ny = int((y1 - y0) / cr.RES) + 1
    grid = np.zeros((nx, ny), dtype=bool)
    for cx, cy, hx, hy in boxes:
        i0 = max(0, int((cx - hx - x0) / cr.RES))
        i1 = min(nx - 1, int((cx + hx - x0) / cr.RES))
        j0 = max(0, int((cy - hy - y0) / cr.RES))
        j1 = min(ny - 1, int((cy + hy - y0) / cr.RES))
        grid[i0:i1 + 1, j0:j1 + 1] = True
    return cr.edt(grid) * cr.RES, x0, y0, nx, ny


# Cartelle in piu' dove cercare i modelli del mondo (quella del mondo originale,
# quando si analizza la copia normalizzata archiviata con la run).
RADICI_EXTRA = []

# Quota del corpo dell'x500 appoggiato (run_multi_exploration.sh): la verita' di
# PX4 ha lo zero alla partenza, gli ostacoli 3D lo hanno al terreno.
QUOTA_APPOGGIO = 0.227


def distanze_3d(mondo_sdf, forza=False):
    """Funzione posizione mondo (Nx3) -> distanza VERA in 3D dall'ostacolo piu'
    vicino, se il generatore del mondo ha salvato gli ostacoli 3D (parco: punti
    di collisione dei modelli + siepi come scatole) o se il mondo contiene mesh o
    modelli inclusi (qualsiasi altro mondo: triangoli di collisione letti da
    geometria_mondo.py); per i mondi di soli box None, e vale la pianta 2D. All'aperto la pianta 2D della fascia 0.4-1.9 m segnava contatto
    sopra il tetto delle auto e sotto le chiome (18 ottobre 2026: 0.00 m in 2D,
    0.50 m in 3D, nessun urto). Il drone e' una sfera di raggio DRONE_RADIUS:
    prudente in verticale, dove l'x500 e' alto ~0.25 m."""
    pianta = os.path.splitext(os.path.realpath(mondo_sdf))[0] + '_ostacoli.npz'
    z = np.load(pianta) if os.path.exists(pianta) else {}
    if 'punti3d' not in z:
        testo = open(mondo_sdf).read()
        if not forza and '<mesh' not in testo and '<include' not in testo:
            return None
        import geometria_mondo as gm
        geo = gm.carica(mondo_sdf, gm.radici_risorse(mondo_sdf, RADICI_EXTRA))
        for avviso in geo.avvisi:
            print('AVVISO geometria:', avviso)
        return geo.distanza
    from scipy.spatial import cKDTree
    albero = cKDTree(z['punti3d'].astype(np.float64))
    scatole = z['scatole3d']

    def distanza(X):
        d, _ = albero.query(X)
        for cx, cy, cz, hx, hy, hz in scatole:
            q = np.maximum(np.abs(X - (cx, cy, cz)) - (hx, hy, hz), 0.0)
            d = np.minimum(d, np.linalg.norm(q, axis=1))
        return d
    return distanza


def box_3d(mondo_sdf):
    """Tutti i box di collisione del mondo: (cx, cy, cz, hx, hy, hz)."""
    text = open(mondo_sdf).read()
    out = []
    for mdl in re.finditer(r'<model name="[^"]*">(.*?)</model>', text, re.S):
        body = mdl.group(1)
        mp = re.match(r'\s*<static>[^<]*</static>\s*<pose>([^<]+)</pose>', body, re.S)
        mpose = [float(v) for v in mp.group(1).split()] if mp else [0.0, 0.0, 0.0]
        for col in re.finditer(
                r'<collision name="[^"]*">\s*<pose>([^<]+)</pose>\s*'
                r'<geometry><box><size>([^<]+)</size></box></geometry>', body, re.S):
            pose = [float(v) for v in col.group(1).split()]
            size = [float(v) for v in col.group(2).split()]
            out.append((mpose[0] + pose[0], mpose[1] + pose[1], mpose[2] + pose[2],
                        size[0] / 2, size[1] / 2, size[2] / 2))
    return out


# Sensore e volo, per decidere quali superfici sono OSSERVABILI.
LIDAR_DZ = 0.25                 # lidar 25 cm sopra il centro del drone (model.sdf)
LIDAR_FOV_V = np.radians(50.0)  # campo verticale +/-50 gradi (gpu_lidar simulato)
LIDAR_PORTATA = 20.0
QUOTE_VOLO = (1.0, 1.35, 1.7)   # fascia dei punti di vista (viewpoint_min/max_z)
MARGINE_VOLO = 0.4              # distanza minima dagli ostacoli per passarci in volo


def _zona_di_volo(mondo_sdf, passo=0.4):
    """Posizioni del sensore da cui il drone puo' guardare: celle a quota di volo
    con almeno MARGINE_VOLO dagli ostacoli, nella regione libera interna piu'
    grande (quella che non tocca il bordo della griglia e' l'interno dell'edificio;
    stanze chiuse e fessure strette restano escluse)."""
    import collections
    dist, x0, y0, nx, ny = griglia_distanze(mondo_sdf)
    libero = dist >= MARGINE_VOLO
    etichetta = np.zeros(libero.shape, np.int32)
    migliore, n_migliore, k = 0, 0, 0
    for si, sj in zip(*np.nonzero(libero)):
        if etichetta[si, sj]:
            continue
        k += 1
        etichetta[si, sj] = k
        q = collections.deque([(si, sj)])
        n, bordo = 0, False
        while q:
            i, j = q.popleft()
            n += 1
            if i == 0 or j == 0 or i == nx - 1 or j == ny - 1:
                bordo = True
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < nx and 0 <= b < ny and libero[a, b] and not etichetta[a, b]:
                    etichetta[a, b] = k
                    q.append((a, b))
        if not bordo and n > n_migliore:
            migliore, n_migliore = k, n
    ii, jj = np.nonzero(etichetta == migliore)
    xs = x0 + (ii + 0.5) * cr.RES
    ys = y0 + (jj + 0.5) * cr.RES
    # diradamento a 'passo' metri
    chiavi = np.unique(np.c_[np.floor(xs / passo), np.floor(ys / passo)].astype(np.int64),
                       axis=0, return_index=True)[1]
    xs, ys = xs[chiavi], ys[chiavi]
    return np.vstack([np.c_[xs, ys, np.full_like(xs, z + LIDAR_DZ)] for z in QUOTE_VOLO])


def campioni_osservabili(mondo_sdf, z_lo=0.2, z_hi=2.0, passo=0.2):
    """Campioni (punti, normali uscenti) sulle facce verticali delle pareti che il
    lidar puo' vedere da almeno un punto della zona di volo.

    La versione precedente (campioni_pareti) contava anche superfici che nessun
    volo puo' vedere - l'interno delle stanze chiuse, le fessure da 30 cm, il
    retro dei muri - e attribuiva il retro di un muro da 20 cm a chi ne aveva
    visto il fronte; in compenso escludeva tutte le pareti perimetrali (il box
    della mappa e' piu' piccolo dell'edificio). Su office_small solo il 60% dei
    suoi campioni era visibile: il 95% era irraggiungibile per costruzione.
    Il risultato si salva in /tmp, perche' i raggi da tracciare sono milioni."""
    import hashlib
    chiave = hashlib.md5(open(mondo_sdf, 'rb').read() + repr(
        (z_lo, z_hi, passo, LIDAR_DZ, LIDAR_FOV_V, LIDAR_PORTATA, QUOTE_VOLO, MARGINE_VOLO)).encode()).hexdigest()[:12]
    cache = f'/tmp/c2x_campioni_osservabili_{chiave}.npz'
    if os.path.exists(cache):
        d = np.load(cache)
        return d['punti'], d['normali']
    import open3d as o3d
    boxes = box_3d(mondo_sdf)
    if not boxes:
        # Mondo fatto solo di mesh (parco): niente pareti a box da campionare,
        # la copertura resta non misurata ('-') invece di far fallire l'analisi.
        return np.zeros((0, 3)), np.zeros((0, 3))

    def dentro_altro(p, escluso):
        for b in boxes:
            if b is not escluso and (abs(p[0] - b[0]) <= b[3] and abs(p[1] - b[1]) <= b[4]
                                     and abs(p[2] - b[2]) <= b[5]):
                return True
        return False

    punti, normali = [], []
    for b in boxes:
        cx, cy, cz, hx, hy, hz = b
        if hz * 2 <= 0.3 or hz * 2 >= 3 or cz - hz >= z_hi or cz + hz <= z_lo:
            continue
        zs = np.arange(max(cz - hz, z_lo), min(cz + hz, z_hi) + 1e-6, passo)
        for sgn in (-1, 1):
            for asse, val, lungo in (('x', cx + sgn * hx, np.arange(cy - hy, cy + hy + 1e-6, passo)),
                                     ('y', cy + sgn * hy, np.arange(cx - hx, cx + hx + 1e-6, passo))):
                n = (sgn, 0.0, 0.0) if asse == 'x' else (0.0, sgn, 0.0)
                for u in lungo:
                    for z in zs:
                        p = (val, u, z) if asse == 'x' else (u, val, z)
                        if dentro_altro((p[0] + 0.05 * n[0], p[1] + 0.05 * n[1], z), b):
                            continue  # faccia nascosta: attaccata a un altro box
                        punti.append(p)
                        normali.append(n)
    punti, normali = np.array(punti), np.array(normali)

    mesh = o3d.geometry.TriangleMesh()
    for cx, cy, cz, hx, hy, hz in boxes:
        m = o3d.geometry.TriangleMesh.create_box(2 * hx, 2 * hy, 2 * hz)
        m.translate((cx - hx, cy - hy, cz - hz))
        mesh += m
    scena = o3d.t.geometry.RaycastingScene()
    scena.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    sensori = _zona_di_volo(mondo_sdf)
    visto = np.zeros(len(punti), bool)
    for j0 in range(0, len(sensori), 40):
        S = sensori[j0:j0 + 40]
        d = punti[:, None, :] - S[None, :, :]
        dist = np.linalg.norm(d, axis=2)
        u = d / dist[..., None]
        davanti = np.einsum('ijk,ik->ij', -u, normali) > 0.05
        utile = davanti & (dist < LIDAR_PORTATA) & (np.abs(np.arcsin(np.clip(u[..., 2], -1, 1))) <= LIDAR_FOV_V)
        utile &= ~visto[:, None]
        ii, jj = np.nonzero(utile)
        if len(ii) == 0:
            continue
        raggi = np.c_[S[jj], u[ii, jj]].astype(np.float32)
        hit = scena.cast_rays(o3d.core.Tensor(raggi))['t_hit'].numpy()
        ok = hit >= dist[ii, jj] - 0.03
        visto[ii[ok]] = True
    punti, normali = punti[visto], normali[visto]
    np.savez(cache, punti=punti, normali=normali)
    return punti, normali


def carica_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('DATA'):
                break
            parts = line.split()
            if parts:
                header[parts[0]] = parts[1:]
        n = int(header['POINTS'][0])
        return np.frombuffer(f.read(n * 12), dtype=np.float32).reshape(n, 3)


def copertura(mappa, campioni, raggio=0.2, dietro=0.1):
    """Frazione dei campioni osservabili con un punto della mappa entro 'raggio'
    E dal lato giusto della parete (non piu' di 'dietro' oltre la faccia): cosi'
    il retro di un muro da 20 cm non conta come visto se si e' visto solo il
    fronte. 'mappa' e' un .pcd o un array Nx3."""
    import open3d as o3d
    if isinstance(mappa, str):
        if not os.path.exists(mappa):
            return None
        mappa = carica_pcd(mappa)
    punti, normali = campioni
    if len(punti) == 0 or len(mappa) == 0:
        return None
    pts = np.asarray(mappa, dtype=np.float64)
    albero = o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)))
    viste = 0
    for p, n in zip(punti, normali):
        k, idx, _ = albero.search_radius_vector_3d(p, raggio)
        if k and np.any((pts[np.asarray(idx)] - p) @ n >= -dietro):
            viste += 1
    return 100.0 * viste / len(punti)


def carica_mesh(ply_path, n=1500000):
    """Punti campionati uniformemente sulla superficie della mappa 3D: i vertici
    da soli non bastano, la decimazione lascia triangoli grandi sulle pareti piane."""
    if not os.path.exists(ply_path):
        return None
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(ply_path)
    if len(m.triangles) == 0:
        return None
    return np.asarray(m.sample_points_uniformly(n).points)


def foto_mappa3d(ply_path):
    """Quota di pavimento e pareti (sotto i 2 m) della mappa 3D colorata dalle foto:
    i vertici mai inquadrati dalla telecamera restano a grigio 0.6 esatto."""
    if not os.path.exists(ply_path):
        return None
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(ply_path)
    v = np.asarray(m.vertices)
    c = np.asarray(m.vertex_colors)
    if len(v) == 0 or len(c) != len(v):
        return None
    sotto = v[:, 2] < 2.0
    grigio = np.all(np.abs(c - 0.6) < 1e-3, axis=1)
    return float(100.0 * (1.0 - grigio[sotto].mean()))


def pavimento(mappa, quota_vera=0.0):
    """Errore VERTICALE della mappa 3D, che precisione() non vede (misura solo la
    distanza orizzontale dalle pareti): quota mediana del pavimento e pendenza
    del piano che lo approssima. Un pavimento a -0.13 m ha rivelato l'origine
    del mondo sbagliata di 12.7 cm; una pendenza di mezzo grado e' l'assetto di
    FAST-LIO che, a 12 m dalla partenza, abbassa la mappa di 15 cm."""
    if mappa is None:
        return None, None
    f = mappa[(mappa[:, 2] > quota_vera - 0.45) & (mappa[:, 2] < quota_vera + 0.25)]
    if len(f) < 1000:
        return None, None
    A = np.c_[f[:, 0], f[:, 1], np.ones(len(f))]
    coef = np.linalg.lstsq(A, f[:, 2], rcond=None)[0]
    pend = float(np.degrees(np.arctan(np.hypot(coef[0], coef[1]))))
    return float(np.median(f[:, 2]) - quota_vera), pend


def precisione(mappa, mondo_sdf, toll=0.15):
    """Quota dei punti di parete (0.3-2.0 m) entro toll da una parete vera.
    'mappa' e' un .pcd o un array Nx3."""
    if isinstance(mappa, str):
        if not os.path.exists(mappa):
            return None
        mappa = carica_pcd(mappa)
    if mappa is None:
        return None
    pareti = [b for b in box_3d(mondo_sdf) if b[5] * 2 < 3 and b[2] - b[5] < 2.0 and b[2] + b[5] > 0.3]
    if not pareti:
        return None  # mondo di sole mesh (parco): non misurabile coi box
    pts = mappa
    pts = pts[(pts[:, 2] > 0.3) & (pts[:, 2] < 2.0)]
    if len(pts) == 0:
        return None
    d = np.full(len(pts), np.inf)
    for cx, cy, _cz, hx, hy, _hz in pareti:
        dx = np.maximum(np.abs(pts[:, 0] - cx) - hx, 0)
        dy = np.maximum(np.abs(pts[:, 1] - cy) - hy, 0)
        d = np.minimum(d, np.sqrt(dx * dx + dy * dy))
    return 100.0 * float(np.mean(d < toll))


def analizza_ulog(path, spawn, dist_info, dist3d=None):
    out = {}
    if ULog is None or not os.path.exists(path):
        return out
    try:
        u = ULog(path, ['vehicle_local_position', 'vehicle_local_position_groundtruth',
                        'vehicle_attitude', 'vehicle_visual_odometry', 'trajectory_setpoint'])
    except Exception:
        return out
    d = {m.name: m.data for m in u.data_list}
    # Buchi della visione esterna (FAST-LIO dentro PX4), solo senza GPS: oltre
    # ~1 s EKF2 perde la posizione e PX4 scende alla cieca ("blind land"); il
    # crash del 5 ottobre 2026 e' seguito a un buco di 2.3 s.
    ev = d.get('vehicle_visual_odometry')
    if ev is not None and len(ev['timestamp']) > 10:
        dt = np.diff(ev['timestamp'] / 1e6)
        out['ev_buco_max'] = float(dt.max())
        out['ev_buchi'] = int((dt > 0.5).sum())
    gt = d.get('vehicle_local_position_groundtruth')
    if gt is None:
        return out
    t = gt['timestamp'] / 1e6
    # Verita' del simulatore: gia' in coordinate MONDO (vedi clearance_report.py).
    wx, wy, alt = np.asarray(gt['y']), np.asarray(gt['x']), -np.asarray(gt['z'])
    # Durata VERA della missione: dal distacco da terra all'appoggio. "volo_s"
    # (tempo sopra 0.9 m) la sottostima: in crociera il drone sta spesso a 0.9-1 m.
    staccato = np.where(alt > 0.2)[0]
    if staccato.size:
        out['missione_s'] = float(t[staccato[-1]] - t[staccato[0]])
    sopra = np.where(alt > 0.9)[0]
    if sopra.size == 0:
        out['volo_s'] = 0.0
        return out
    crociera = slice(sopra[0], sopra[-1])
    out['volo_s'] = t[sopra[-1]] - t[sopra[0]]
    out['quota_min'] = float(alt[crociera].min())
    v = np.hypot(np.asarray(gt['vx']), np.asarray(gt['vy']))
    out['vel_max'] = float(v[crociera].max())
    out['vel_media'] = float(v[crociera].mean())
    # DONDOLIO: quota dei comandi in crociera con velocita' esattamente zero (il
    # riferimento fermo in attesa del piano nuovo) e scarto mediano fra velocita'
    # comandata e velocita' vera. cubicle_office a 1.1 m/s, prima della
    # correzione: 34% e 0.56 m/s; office_small 0.9 m/s: 0.14 m/s.
    sp = d.get('trajectory_setpoint')
    if sp is not None and len(sp['timestamp']) > 10:
        tsp = sp['timestamp'] / 1e6
        m = (tsp > t[crociera][0] + 20) & (tsp < t[crociera][-1] - 5)
        if m.sum() > 10:
            vr = np.hypot(sp['velocity[0]'][m], sp['velocity[1]'][m])
            out['rif_fermo'] = float(100.0 * np.mean(vr == 0))
            vx = np.interp(tsp[m], t, np.asarray(gt['vx']))
            vy = np.interp(tsp[m], t, np.asarray(gt['vy']))
            out['scarto_vel'] = float(np.median(np.hypot(sp['velocity[0]'][m] - vx,
                                                         sp['velocity[1]'][m] - vy)))
    # Dove si e' appoggiato rispetto al punto di decollo (verita' del simulatore).
    out['dist_casa'] = float(np.hypot(wx[-1] - wx[0], wy[-1] - wy[0]))

    air = alt > 0.4
    if air.any() and (dist_info is not None or dist3d is not None):
        if dist3d is not None:
            c = dist3d(np.c_[wx[air], wy[air], alt[air] + QUOTA_APPOGGIO])
        else:
            dist, x0, y0, nx, ny = dist_info
            i = np.clip(((wx[air] - x0) / cr.RES).astype(int), 0, nx - 1)
            j = np.clip(((wy[air] - y0) / cr.RES).astype(int), 0, ny - 1)
            c = dist[i, j]
        out['margine_min'] = float(c.min())
        out['sotto_raggio_pct'] = float(100.0 * (c < cr.DRONE_RADIUS).mean())

    att = d.get('vehicle_attitude')
    if att is not None:
        q = np.stack([att['q[0]'], att['q[1]'], att['q[2]'], att['q[3]']], axis=1)
        w, x, y, z = q.T
        roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
        out['incl_max'] = float(np.maximum(np.abs(roll), np.abs(pitch)).max())

    lp = d.get('vehicle_local_position')
    if lp is not None:
        tl = lp['timestamp'] / 1e6
        # Stima ancorata allo spawn, verita' in coordinate mondo: si toglie lo
        # spawn alla verita' (NED: x=nord=y mondo, y=est=x mondo).
        gx = np.interp(tl, t, np.asarray(gt['x'])) - spawn[1]
        gy = np.interp(tl, t, np.asarray(gt['y'])) - spawn[0]
        a = np.interp(tl, t, alt)
        m = a > 0.9
        if m.any():
            e = np.hypot(np.asarray(lp['x'])[m] - gx[m], np.asarray(lp['y'])[m] - gy[m])
            out['err_stima_med'] = float(np.median(e))
            out['err_stima_max'] = float(e.max())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cartella', nargs='?', default='/tmp/c2x_reliability')
    ap.add_argument('--mondo', default=None,
                    help='nome di un mondo del progetto o percorso di un .sdf; di serie il '
                         'mondo archiviato con la run (rel_mondo.sdf), poi office_small')
    ap.add_argument('--spawn', default=None,
                    help='partenza del drone 1 "x,y"; di serie da rel_mondo.json')
    a = ap.parse_args()

    # Le run archiviate dopo ottobre 2026 portano con se' il mondo volato.
    import json
    info = {}
    for f in sorted(glob.glob(os.path.join(a.cartella, 'rel_mondo*.json'))):
        info = json.load(open(f))
        break
    if a.mondo:
        mondo = a.mondo if os.path.isfile(a.mondo) else os.path.join(
            QUI, '..', 'src', 'c2x_worlds', 'worlds', a.mondo + '.sdf')
    elif os.path.exists(os.path.join(a.cartella, 'rel_mondo.sdf')):
        mondo = os.path.join(a.cartella, 'rel_mondo.sdf')
        # i mondi del progetto hanno accanto la pianta degli ostacoli: si usa l'originale
        if info.get('origine') and os.path.exists(info['origine']):
            orig = info['origine']
            if os.path.exists(os.path.splitext(orig)[0] + '_ostacoli.npz') or info.get('tarato'):
                mondo = orig
    else:
        mondo = os.path.join(QUI, '..', 'src', 'c2x_worlds', 'worlds', 'office_small.sdf')
    if info.get('origine'):
        RADICI_EXTRA.append(os.path.dirname(info['origine']))
    if a.spawn:
        spawn = [float(s) for s in a.spawn.split(',')]
    elif info.get('partenze'):
        spawn = [float(s) for s in info['partenze'][0].split(',')[:2]]
    else:
        spawn = [-3.5, -3.0]
    print(f'mondo: {mondo}  partenza drone 1: {spawn}')
    # Mondi non tarati (qualsiasi .sdf, preparato da prepara_mondo.py): margine 3D
    # dai triangoli di collisione. Per quelli tarati la misura resta quella delle
    # campagne precedenti, per confrontabilita'.
    generico = bool(info) and not info.get('tarato', True) and not a.mondo
    dist_info = None if generico else griglia_distanze(mondo)
    dist3d = distanze_3d(mondo, forza=generico)
    campioni = campioni_osservabili(mondo)

    runs = sorted({int(m.group(1)) for f in glob.glob(os.path.join(a.cartella, 'rel_run_*.log'))
                   for m in [re.search(r'rel_run_(\d+)\.log', f)] if m})
    if not runs:
        print('nessuna run in', a.cartella)
        return 1

    righe = []
    for n in runs:
        run = open(os.path.join(a.cartella, f'rel_run_{n}.log'), errors='replace').read()
        p = os.path.join(a.cartella, f'rel_expl_{n}.log')
        expl = open(p, errors='replace').read() if os.path.exists(p) else ''
        r = {'run': n}
        if conta(expl, 'CRASH DETECTED'):
            r['esito'] = 'CRASH'
        elif conta(expl, 'DISARMATO IN VOLO|VOLO INTERROTTO'):
            r['esito'] = 'DISARMO'
        elif conta(expl, 'TEMPO MASSIMO'):
            r['esito'] = 'TETTO'
        elif 'Esplorazione completata' in run:
            r['esito'] = 'finita'
        elif 'Timeout raggiunto' in run:
            r['esito'] = 'TETTO'
        elif 'STALLO' in run or not expl:
            r['esito'] = 'STALLO/NON PARTITA'
        else:
            r['esito'] = 'INTERROTTA'
        m = re.findall(r'has finished exploration \(frontiere note (\d+)', expl)
        r['frontiere'] = int(m[-1]) if m else None
        r['atterrato'] = 'rientrati e atterrati' in run
        m = re.findall(r'\((\d+) punti totali', run)
        r['punti'] = int(m[-1]) if m else None
        m = re.findall(r'volume esplorato ([\d.]+) m3', expl)
        r['volume'] = float(m[-1]) if m else None
        r['plan_fail'] = conta(expl, r'Plan fail')
        r['stop_prox'] = conta(expl, r'PROXIMITY STOP')
        r['clamp'] = conta(expl, r'clamped to')
        r['filtro'] = conta(expl, r'\[Filtro\]')
        r['limiti'] = conta(expl, r'\[Limiti\]')
        r['watchdog'] = conta(expl, r'gained only')
        r['segfault'] = conta(expl, r'Segmentation fault|Aborted \(')
        r.update(analizza_ulog(os.path.join(a.cartella, f'rel_ulog_{n}_0.ulg'), spawn, dist_info, dist3d))
        r['copertura'] = copertura(os.path.join(a.cartella, f'rel_map_{n}.pcd'), campioni)
        r['precisione'] = precisione(os.path.join(a.cartella, f'rel_map_{n}.pcd'), mondo)
        # Mappa realistica 3D (il prodotto finale): stessa misura sulla superficie.
        m3d = carica_mesh(os.path.join(a.cartella, f'rel_mappa3d_{n}_1', 'mappa_realistica.ply'))
        r['cop3d'] = copertura(m3d, campioni) if m3d is not None else None
        r['prec3d'] = precisione(m3d, mondo) if m3d is not None else None
        r['pav'], r['pend'] = pavimento(m3d)
        r['foto'] = foto_mappa3d(os.path.join(a.cartella, f'rel_mappa3d_{n}_1', 'mappa_realistica.ply'))
        righe.append(r)

    def f(v, fmt):
        return '-' if v is None else format(v, fmt)

    print(f'{"run":>3} {"esito":>8} {"front":>5} {"vol m3":>6} {"cop%":>5} {"prec%":>5} {"cop3d":>5} {"pre3d":>5} {"att.":>4} {"miss.s":>6} {"marg.m":>6} '
          f'{"<raggio":>7} {"q.min":>5} {"incl":>5} {"vel":>5} {"v.med":>5} {"err.st":>6} '
          f'{"pfail":>5} {"stop":>4} {"filt":>4} {"lim":>4} {"clamp":>5} {"wdog":>4} {"ev>0.5s":>7} {"ev max":>6} '
          f'{"pav m":>6} {"pend":>5} {"casa m":>6} {"rif0%":>5} {"dv":>5} {"foto%":>5}')
    for r in righe:
        print(f'{r["run"]:>3} {r["esito"]:>8} {f(r["frontiere"], "d"):>5} '
              f'{f(r.get("volume"), ".0f"):>6} {f(r.get("copertura"), ".0f"):>5} '
              f'{f(r.get("precisione"), ".0f"):>5} '
              f'{f(r.get("cop3d"), ".1f"):>5} {f(r.get("prec3d"), ".1f"):>5} '
              f'{("si" if r["atterrato"] else "NO"):>4} {f(r.get("missione_s"), ".0f"):>6} '
              f'{f(r.get("margine_min"), ".2f"):>6} {f(r.get("sotto_raggio_pct"), ".1f"):>6}% '
              f'{f(r.get("quota_min"), ".2f"):>5} {f(r.get("incl_max"), ".0f"):>5} '
              f'{f(r.get("vel_max"), ".2f"):>5} {f(r.get("vel_media"), ".2f"):>5} '
              f'{f(r.get("err_stima_med"), ".2f"):>6} '
              f'{r["plan_fail"]:>5} {r["stop_prox"]:>4} {r["filtro"]:>4} {r["limiti"]:>4} '
              f'{r["clamp"]:>5} {r["watchdog"]:>4} {f(r.get("ev_buchi"), "d"):>7} '
              f'{f(r.get("ev_buco_max"), ".2f"):>6} {f(r.get("pav"), "+.2f"):>6} '
              f'{f(r.get("pend"), ".2f"):>5} {f(r.get("dist_casa"), ".2f"):>6} '
              f'{f(r.get("rif_fermo"), ".0f"):>5} {f(r.get("scarto_vel"), ".2f"):>5} '
              f'{f(r.get("foto"), ".1f"):>5}')

    ok = [r for r in righe if r['esito'] == 'finita' and r['atterrato']]
    # PULITA = conclusa, atterrata, senza contatti con gli ostacoli e senza
    # inclinazioni da urto. E' la sola che conta come robusta: la run BASE_GPS 2
    # risultava 'finita e atterrata' dopo aver colpito una parete a 76 gradi.
    # ... e con copertura delle pareti almeno COPERTURA_MIN: una run che
    # dichiara concluso dopo dieci secondi (V3_LIO_1: 60%) non e' un successo.
    # Ottobre 2026: la copertura si misura sulle sole superfici OSSERVABILI dalla
    # zona di volo (vedi campioni_osservabili); la soglia vale per la mappa 3D
    # finale se c'e', altrimenti per la mappa del planner. Con la misura vecchia
    # la soglia era 70% e il tetto fisico ~73%.
    COPERTURA_MIN = 95.0

    def copertura_finale(r):
        return r.get('cop3d') if r.get('cop3d') is not None else r.get('copertura')
    pulite = [r for r in ok if (r.get('sotto_raggio_pct') or 0) == 0
              and (r.get('incl_max') or 0) < 45
              and (copertura_finale(r) is None or copertura_finale(r) >= COPERTURA_MIN)]
    print()
    print('cop% / prec%: mappa del planner; cop3d / pre3d: mappa realistica 3D. Copertura = '
          'quota delle superfici di parete VISIBILI dalla zona di volo; precisione = punti di '
          'parete entro 15 cm da una parete vera. ev>0.5s / ev max: buchi della stima di '
          'FAST-LIO verso PX4. pav / pend: quota (vero 0) e pendenza in gradi del pavimento '
          'della mappa 3D.')
    print(f'RUN PULITE (concluse, atterrate, nessun contatto, inclinazione < 45, '
          f'copertura finale >= {COPERTURA_MIN:.0f}%): {len(pulite)}/{len(righe)}')
    print(f'RIEPILOGO: {len(ok)}/{len(righe)} run concluse E atterrate; '
          f'crash {sum(r["esito"] == "CRASH" for r in righe)}; '
          f'tetto {sum(r["esito"] == "TETTO" for r in righe)}; '
          f'contatti (margine < {cr.DRONE_RADIUS} m) in '
          f'{sum((r.get("sotto_raggio_pct") or 0) > 0 for r in righe)} run')
    volo = [r['missione_s'] for r in ok if r.get('missione_s')]
    if volo:
        print(f'           durata della missione (decollo-appoggio) nelle run riuscite: mediana '
              f'{np.median(volo):.0f} s (min {min(volo):.0f}, max {max(volo):.0f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
