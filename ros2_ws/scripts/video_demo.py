#!/usr/bin/env python3
"""VIDEO DIMOSTRATIVO di un volo (MP4 1920x1080), dai dati archiviati di una run.

Sequenza: titolo -> replay del volo accelerato (pianta che si costruisce, spazio
visitato, traiettoria, telecamera di bordo, numeri dal vivo) -> mappa 3D
ricostruita in orbita con la traiettoria -> schermata finale con i risultati.

Tutto e' etichettato come SIMULAZIONE (Gazebo + PX4 SITL). La pianta che "si
costruisce" e' una ricostruzione a posteriori: ogni cella compare quando, lungo la
traiettoria vera, entra nella portata del lidar (10 m) con la vista libera sulla
geometria vera del mondo; lo spazio visitato e' quello di copertura_visita.py
(entro 2 m dal drone, vista libera). La mappa 3D e' quella prodotta dalla run.

Uso: video_demo.py <cartella_campagna> <indice_run> <uscita.mp4>
         [--mondo cubicle_office] [--spawn=-13.5,-4] [--velocita 12] [--titolo "..."]
Serve GStreamer con x264enc e mp4mux (gst-launch-1.0).
"""
import argparse
import glob
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import clearance_report as cr  # noqa: E402
import copertura_visita as cv  # noqa: E402

RES = 0.05            # pianta a 5 cm
PORTATA_LIDAR = 20.0  # m: sdf_map.max_ray_length, la portata con cui il pianificatore costruisce la mappa
RAGGIO_VISITA = 2.0
W, H = 1920, 1080
FPS = 60   # cambiabile con --fps

# Jetson (aarch64, Open3D 0.20 = Filament su Vulkan): il renderer fuori schermo
# restituisce immagini TUTTE NERE oltre un certo carico. Misurato: mesh della mappa
# 3D da 1.25 M triangoli nera a ogni risoluzione; 200k triangoli a 1920x1080 nera
# 2 volte su 2; 300k a 1280x720 giusta 2 volte su 2. Li' si rende a 1280x720 e si
# ingrandisce a W x H, con la mappa ridotta a 250k triangoli. Sul PC x86 nulla cambia.
ARM = platform.machine() in ('aarch64', 'arm64')
RENDER_W, RENDER_H = (1280, 720) if ARM else (W, H)
MAX_TRIANGOLI_RENDER = 250000 if ARM else None


def riduci_per_render(mesh):
    """Sul Jetson riduce la mesh a MAX_TRIANGOLI_RENDER triangoli (vedi sopra)."""
    if MAX_TRIANGOLI_RENDER and len(mesh.triangles) > MAX_TRIANGOLI_RENDER:
        mesh = mesh.simplify_quadric_decimation(MAX_TRIANGOLI_RENDER)
        mesh.compute_vertex_normals()
    return mesh


_ultimo_buono = None
_neri = 0


def rendi(r):
    """Fotogramma del renderer fuori schermo come immagine PIL W x H.

    Sul Jetson un render che arriva dopo una pausa (qui: scritte e JPEG del
    fotogramma prima, ~0.25 s) esce tutto nero nel 20-40% dei casi; i tentativi
    subito dopo riescono quasi sempre. Misurato su 300 fotogrammi del replay: con 3
    tentativi 12 restavano neri, con 12 tentativi 1. Quell'uno prende l'ultimo
    fotogramma buono (un fotogramma fermo non si vede, un lampo nero si')."""
    global _ultimo_buono, _neri
    from PIL import Image
    # Prima del primo fotogramma buono il motore sta ancora caricando la scena: si
    # insiste di piu' (il primo fotogramma e' l'anteprima del video).
    for _ in range(12 if _ultimo_buono is not None else 30):
        a = np.asarray(r.render_to_image())
        if a.max() > 0:
            _ultimo_buono = a
            break
    else:
        _neri += 1
        if _neri % 10 == 1:
            print(f'[video] fotogrammi 3D neri rimpiazzati dal precedente: {_neri}', flush=True)
        if _ultimo_buono is not None:
            a = _ultimo_buono
    im = Image.fromarray(a)
    return im if im.size == (W, H) else im.resize((W, H), Image.LANCZOS)


# ---------------------------------------------------------------- dati del volo
def carica_volo(ulg):
    from pyulog import ULog
    u = ULog(ulg, ['vehicle_local_position_groundtruth', 'vehicle_attitude_groundtruth'])
    d = {m.name: m.data for m in u.data_list}
    g, a = d['vehicle_local_position_groundtruth'], d['vehicle_attitude_groundtruth']
    tg = g['timestamp'].astype(np.float64) / 1e6
    ta = a['timestamp'].astype(np.float64) / 1e6
    q0, q1, q2, q3 = a['q[0]'], a['q[1]'], a['q[2]'], a['q[3]']
    yaw_ned = np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)))
    T = np.arange(tg[0], tg[-1], 0.1)
    x, y, z = (np.interp(T, tg, g[k]) for k in ('y', 'x', 'z'))
    vx, vy = np.interp(T, tg, g['vy']), np.interp(T, tg, g['vx'])
    yaw = np.pi / 2 - np.interp(T, ta, yaw_ned)      # ENU, da est in senso antiorario
    return dict(t=T, x=x, y=y, z=-z, v=np.hypot(vx, vy), yaw=yaw)


def offset_log(volo, log):
    """Scarto fra l'orologio dei log ROS e quello del log PX4 (posizioni [StuckDiag])."""
    T, P = [], []
    for l in open(log, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\].*StuckDiag\] pos\((\S+) (\S+) (\S+)\)', l)
        if m:
            T.append(float(m.group(1)))
            P.append((float(m.group(2)), float(m.group(3))))
    T, P = np.array(T), np.array(P)
    tg, gx, gy = volo['t'], volo['x'], volo['y']

    def errore(off):
        return np.median(np.hypot(np.interp(T - off, tg, gx) - P[:, 0], np.interp(T - off, tg, gy) - P[:, 1]))
    grezzo = np.arange(T[0] - tg[-1], T[-1] - tg[0], 0.5)
    o = grezzo[np.argmin([errore(v) for v in grezzo])]
    fine = np.arange(o - 1, o + 1, 0.02)
    return fine[np.argmin([errore(v) for v in fine])]


def eventi(log, off):
    chiavi = {
        'via': r'giro iniziale concluso',
        'fine_esplorazione': r'has finished exploration',
        'fine_visita': r'fase conclusa',
        'sopra_casa': r'\[Casa\] sopra il punto di decollo',
    }
    out = {}
    for l in open(log, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\]', l)
        if not m:
            continue
        for k, rx in chiavi.items():
            if k not in out and re.search(rx, l):
                out[k] = float(m.group(1)) - off
    return out


def abbina_fotogrammi(cartella, volo):
    info = json.load(open(os.path.join(cartella, 'fotogrammi', 'pose.json')))['fotogrammi']
    tg, gx, gy, gz, gyaw = volo['t'], volo['x'], volo['y'], volo['z'], volo['yaw']
    tempi, k = [], 0
    for f in info:
        p = np.array(f['t'])
        R = np.array(f['R'])
        yaw_cam = math.atan2(R[1, 0], R[0, 0])   # asse x della telecamera (avanti) nel mondo
        # in avanti nel tempo: il fotogramma successivo non precede il precedente.
        # Posizione E orientamento: nel giro iniziale il drone e' fermo e la sola
        # posizione non basta a datare i fotogrammi.
        fin = min(len(tg), k + 600)
        dyaw = np.abs(np.angle(np.exp(1j * (gyaw[k:fin] - yaw_cam))))
        # (la quota no: e' quella stimata a bordo della telecamera, ~25 cm sopra il
        # corpo, e spingeva l'abbinamento verso istanti piu' tardi)
        d = np.hypot(gx[k:fin] - p[0], gy[k:fin] - p[1]) + 0.5 * dyaw
        # Fra i candidati quasi buoni quanto il migliore, il PRIMO: i fotogrammi
        # si scattano ogni 0.3 m o 10 gradi, quindi il successivo e' vicino nel
        # tempo. Con il solo minimo, un fotogramma a imbardata 0 all'inizio del
        # giro iniziale finiva alla fine del giro e trascinava con se' gli altri.
        j = k + int(np.nonzero(d <= d.min() + 0.08)[0][0]) if fin > k else k
        tempi.append(tg[j])
        k = j
    return [os.path.join(cartella, 'fotogrammi', f['file']) for f in info], np.array(tempi)


def pianta_mondo(mondo):
    sdf = os.path.join(QUI, '..', 'src', 'c2x_worlds', 'worlds', mondo + '.sdf')
    boxes = cr.boxes_from_sdf(sdf, 0.4, 1.6)
    x0, y0, x1, y1 = -16.5, -16.5, 16.5, 16.5
    nx, ny = int((x1 - x0) / RES), int((y1 - y0) / RES)
    occ = np.zeros((nx, ny), dtype=bool)
    for cx, cy, hx, hy in boxes:
        i0, i1 = max(0, int((cx - hx - x0) / RES)), min(nx - 1, int((cx + hx - x0) / RES))
        j0, j1 = max(0, int((cy - hy - y0) / RES)), min(ny - 1, int((cy + hy - y0) / RES))
        occ[i0:i1 + 1, j0:j1 + 1] = True
    return occ, x0, y0


def istanti_visti(occ, x0, y0, volo, t_da, t_a, portata, solo_liberi=False):
    """Primo istante in cui ogni cella e' entro 'portata' dal drone con vista libera."""
    nx, ny = occ.shape
    visto = np.full(occ.shape, np.inf)
    ang = np.linspace(0, 2 * np.pi, 1440 if portata > 5 else 720, endpoint=False)
    passi = np.arange(0.0, portata, RES)
    cs, sn = np.cos(ang)[:, None], np.sin(ang)[:, None]
    sel = np.nonzero((volo['t'] >= t_da) & (volo['t'] <= t_a) & (volo['z'] > 0.5))[0][::2]
    for k in sel:
        px, py = volo['x'][k], volo['y'][k]
        I = ((px + cs * passi - x0) / RES).astype(int)
        J = ((py + sn * passi - y0) / RES).astype(int)
        fuori = (I < 0) | (I >= nx) | (J < 0) | (J >= ny)
        I, J = np.clip(I, 0, nx - 1), np.clip(J, 0, ny - 1)
        muro = occ[I, J] | fuori
        primo = np.where(muro.any(1), muro.argmax(1), len(passi))
        if solo_liberi:
            dentro = np.arange(len(passi))[None, :] < primo[:, None]
        else:
            dentro = np.arange(len(passi))[None, :] <= primo[:, None]
        dentro &= ~fuori
        ii, jj = I[dentro], J[dentro]
        nuove = np.isinf(visto[ii, jj])
        visto[ii[nuove], jj[nuove]] = volo['t'][k]
    return visto


def prepara(a):
    run_dir, i = a.cartella, a.run
    cache = os.path.join(run_dir, f'video_dati_{i}.npz')
    if os.path.exists(cache) and not a.rifai:
        d = dict(np.load(cache, allow_pickle=True))
        d['fotogrammi'] = list(d['fotogrammi'])
        d['eventi'] = d['eventi'].item()
        return d
    print('[video] lettura del volo...', flush=True)
    volo = carica_volo(os.path.join(run_dir, f'rel_ulog_{i}_0.ulg'))
    log = os.path.join(run_dir, f'rel_expl_{i}.log')
    off = offset_log(volo, log)
    ev = eventi(log, off)
    aria = np.nonzero(volo['z'] > 0.3)[0]
    ev['decollo'], ev['appoggio'] = volo['t'][aria[0]], volo['t'][aria[-1]]
    print(f'[video] eventi (s dal decollo): ' + ', '.join(
        f'{k} {v - ev["decollo"]:.0f}' for k, v in sorted(ev.items(), key=lambda kv: kv[1])), flush=True)
    foto, tfoto = abbina_fotogrammi(os.path.join(run_dir, f'rel_mappa3d_{i}_1'), volo)
    print(f'[video] {len(foto)} fotogrammi abbinati', flush=True)
    occ, x0, y0 = pianta_mondo(a.mondo)
    print('[video] pianta che si costruisce (portata lidar)...', flush=True)
    visto = istanti_visti(occ, x0, y0, volo, ev['decollo'], ev['appoggio'], PORTATA_LIDAR)
    visitato = np.full(occ.shape, np.inf)   # non piu' mostrato: la pianta mostra cio' che vede il LiDAR
    dist = cr.edt(occ) * RES
    sx, sy = (float(v) for v in a.spawn.split(','))
    area = cv.raggiungibili(dist >= 0.5, int((sx - x0) / RES), int((sy - y0) / RES))
    d = dict(volo_t=volo['t'], volo_x=volo['x'], volo_y=volo['y'], volo_z=volo['z'], volo_v=volo['v'],
             volo_yaw=volo['yaw'], fotogrammi=np.array(foto), t_foto=tfoto, occ=occ, x0=x0, y0=y0,
             visto=visto, visitato=visitato, area=area, eventi=np.array(ev, dtype=object))
    np.savez_compressed(cache, **d)
    d['fotogrammi'] = list(d['fotogrammi'])
    d['eventi'] = ev
    return d


# ---------------------------------------------------------------- disegno
MPL_FONT = os.path.expanduser('~/.local/lib/python3.12/site-packages/matplotlib/mpl-data/fonts/ttf')
BG = (0, 0, 0)
ARANCIO = (255, 159, 28)
TEAL = (22, 104, 98)
GRIGIO = (150, 165, 185)


def font(dim, grassetto=False):
    from PIL import ImageFont
    nome = 'DejaVuSans-Bold.ttf' if grassetto else 'DejaVuSans.ttf'
    for base in (MPL_FONT, '/usr/share/fonts/truetype/dejavu'):
        p = os.path.join(base, nome)
        if os.path.exists(p):
            return ImageFont.truetype(p, dim)
    return ImageFont.load_default()


def mmss(s):
    s = max(0, int(round(s)))
    return f'{s // 60:02d}:{s % 60:02d}'


class Replay:
    MAPPA = (40, 104, 960)          # x, y, lato del riquadro della pianta
    CAMERA = (1040, 104, 840, 630)  # x, y, w, h

    def __init__(self, d, velocita, titolo, sottotitolo):
        from PIL import Image
        self.d, self.vel, self.titolo, self.sottotitolo = d, velocita, titolo, sottotitolo
        self.ev = d['eventi']
        occ = d['occ']
        self.nx, self.ny = occ.shape
        lato = self.MAPPA[2]
        self.scala = lato / (self.nx * RES)      # px per metro
        # indici di campionamento pianta -> immagine (righe = y decrescente)
        cols = np.arange(lato)
        righe = np.arange(lato)
        self.ii = np.clip((cols / lato * self.nx).astype(int), 0, self.nx - 1)
        self.jj = np.clip(((lato - 1 - righe) / lato * self.ny).astype(int), 0, self.ny - 1)
        self.occ_img = occ[np.ix_(self.ii, self.jj)].T
        self.visto_img = d['visto'][np.ix_(self.ii, self.jj)].T
        self.visit_img = d['visitato'][np.ix_(self.ii, self.jj)].T
        self.area = d['area']
        self.n_area = max(1, int(self.area.sum()))
        # area mappata: celle percorribili gia' viste dal LiDAR, in ordine di tempo
        self.visto_area_t = np.sort(d['visto'][self.area])
        self.occ = occ
        ang = np.linspace(0, 2 * np.pi, 720, endpoint=False)
        self.scan_cs, self.scan_sn = np.cos(ang)[:, None], np.sin(ang)[:, None]
        self.scan_passi = np.arange(0.2, PORTATA_LIDAR, RES)
        self.foto_cache = (None, None)
        self.Image = Image
        self.f_tit, self.f_sub = font(36, True), font(20)
        self.f_lab, self.f_num, self.f_chip, self.f_small = font(15, True), font(40, True), font(15, True), font(15)
        t = d['volo_t']
        self.t, self.x, self.y, self.yaw = t, d['volo_x'], d['volo_y'], d['volo_yaw']
        # velocita' mostrata: media su 1 s (quella istantanea saltava a ogni sosta)
        self.v = np.convolve(d['volo_v'], np.ones(10) / 10, mode='same')
        passo = np.r_[0, np.hypot(np.diff(self.x), np.diff(self.y))]
        passo[d['volo_z'] < 0.3] = 0
        self.dist = np.cumsum(passo)

    def stato(self, t):
        """Posizione, imbardata e velocita' interpolate (i dati sono a 10 Hz: a 60 fps
        e 5x il campione piu' vicino si ripeteva e il drone avanzava a scatti)."""
        x = float(np.interp(t, self.t, self.x))
        y = float(np.interp(t, self.t, self.y))
        c = float(np.interp(t, self.t, np.cos(self.yaw)))
        s_ = float(np.interp(t, self.t, np.sin(self.yaw)))
        v = float(np.interp(t, self.t, self.v))
        dist = float(np.interp(t, self.t, self.dist))
        z = float(np.interp(t, self.t, self.d['volo_z']))
        return x, y, math.atan2(s_, c), v, dist, z

    def scansione(self, x, y):
        """Punti colpiti dai raggi del LiDAR alla quota di volo (720 raggi, portata di mappa)."""
        x0, y0 = self.d['x0'], self.d['y0']
        I = ((x + self.scan_cs * self.scan_passi - x0) / RES).astype(int)
        J = ((y + self.scan_sn * self.scan_passi - y0) / RES).astype(int)
        fuori = (I < 0) | (I >= self.nx) | (J < 0) | (J >= self.ny)
        I, J = np.clip(I, 0, self.nx - 1), np.clip(J, 0, self.ny - 1)
        muro = self.occ[I, J] | fuori
        colpito = muro.any(1)
        k = np.where(colpito, muro.argmax(1), len(self.scan_passi) - 1)
        r = self.scan_passi[k]
        px = x + self.scan_cs[:, 0] * r
        py = y + self.scan_sn[:, 0] * r
        return px, py, colpito

    def px(self, x, y):
        X0, Y0, lato = self.MAPPA
        return (X0 + (x - self.d['x0']) * self.scala, Y0 + lato - (y - self.d['y0']) * self.scala)

    def fase(self, t):
        ev = self.ev
        if t < ev.get('via', ev['decollo'] + 15):
            return 0
        if t < ev.get('fine_esplorazione', 1e18):
            return 1
        if 'fine_visita' in ev and t < ev['fine_visita']:
            return 2
        return 3

    def frame(self, t):
        from PIL import ImageDraw
        img = self.Image.new('RGB', (W, H), BG)
        # --- pianta
        X0, Y0, lato = self.MAPPA
        rgb = np.empty((lato, lato, 3), np.uint8)
        rgb[:] = (17, 23, 33)
        visto = self.visto_img <= t
        rgb[visto & ~self.occ_img] = (40, 56, 82)
        rgb[visto & self.occ_img] = (225, 232, 242)
        img.paste(self.Image.fromarray(rgb), (X0, Y0))
        dr = ImageDraw.Draw(img, 'RGBA')
        dr.rectangle([X0 - 1, Y0 - 1, X0 + lato, Y0 + lato], outline=(45, 58, 78), width=1)
        k = int(np.searchsorted(self.t, t))
        k = min(max(k, 0), len(self.t) - 1)
        sx, sy, syaw, sv, sdist, sz = self.stato(t)
        # scansione LiDAR attuale: area vista (azzurro trasparente) e punti colpiti
        if sz > 0.3:
            hx_, hy_, colpito = self.scansione(sx, sy)
            poligono = [self.px(a, b) for a, b in zip(hx_, hy_)]
            dr.polygon(poligono, fill=(0, 190, 255, 38))
            for a, b in zip(hx_[colpito], hy_[colpito]):
                u, w = self.px(a, b)
                dr.ellipse([u - 1.6, w - 1.6, u + 1.6, w + 1.6], fill=(0, 225, 255, 255))
        k0 = int(np.searchsorted(self.t, self.ev['decollo']))
        # casa
        hx, hy = self.px(self.x[k0], self.y[k0])
        dr.rectangle([hx - 9, hy - 9, hx + 9, hy + 9], outline=(240, 240, 240), width=2)
        dr.text((hx + 12, hy - 9), 'take-off', font=self.f_small, fill=(240, 240, 240))
        # traiettoria: vecchia tenue, ultimi 20 s piena
        if k > k0 + 1:
            vecchi = list(range(k0, max(k0, k - 200), 5)) + [max(k0, k - 200)]
            pts = [self.px(self.x[i], self.y[i]) for i in vecchi]
            if len(pts) > 1:
                dr.line(pts, fill=ARANCIO + (160,), width=2)
            recenti = [self.px(self.x[i], self.y[i]) for i in range(max(k0, k - 200), k, 2)]
            recenti.append(self.px(sx, sy))
            if len(recenti) > 1:
                dr.line(recenti, fill=ARANCIO + (255,), width=3)
        # drone (posizione interpolata) con il verso di marcia
        cx, cy = self.px(sx, sy)
        dr.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], fill=ARANCIO, outline=(255, 255, 255), width=2)
        dr.line([(cx, cy), (cx + 16 * math.cos(syaw), cy - 16 * math.sin(syaw))], fill=(255, 255, 255), width=3)
        # barra di scala e legenda
        bx, by = X0 + 24, Y0 + lato - 30
        dr.line([(bx, by), (bx + 5 * self.scala, by)], fill=(230, 230, 230), width=3)
        dr.text((bx, by - 22), '5 m', font=self.f_small, fill=(230, 230, 230))
        lx, ly = X0 + lato - 290, Y0 + lato - 116
        dr.rectangle([lx - 12, ly - 10, X0 + lato - 12, Y0 + lato - 12], fill=(0, 0, 0, 200))
        for n, (col, txt) in enumerate([((225, 232, 242), 'walls detected by the LiDAR'),
                                        ((40, 56, 82), 'area observed by the LiDAR'),
                                        ((0, 200, 255), 'current LiDAR scan'),
                                        (ARANCIO, 'flight path')]):
            dr.rectangle([lx, ly + n * 24, lx + 16, ly + n * 24 + 14], fill=col)
            dr.text((lx + 26, ly + n * 24 - 2), txt, font=self.f_small, fill=(220, 226, 235))
        # --- telecamera di bordo
        CX, CY, CW, CH = self.CAMERA
        idx = int(np.searchsorted(self.d['t_foto'], t, side='right')) - 1
        if idx >= 0:
            if self.foto_cache[0] != idx:
                im = self.Image.open(self.d['fotogrammi'][idx]).convert('RGB').resize((CW, CH))
                self.foto_cache = (idx, im)
            img.paste(self.foto_cache[1], (CX, CY))
        else:
            dr.rectangle([CX, CY, CX + CW, CY + CH], fill=(20, 26, 36))
        dr.rectangle([CX, CY, CX + 250, CY + 34], fill=(11, 16, 24, 190))
        dr.text((CX + 12, CY + 8), 'ONBOARD CAMERA', font=self.f_lab, fill=(235, 240, 248))
        # --- pannello dei numeri
        PX, PY, PW, PH = CX, CY + CH + 20, CW, H - (CY + CH + 20) - 16
        dr.rounded_rectangle([PX, PY, PX + PW, PY + PH], radius=14, fill=(20, 28, 40))
        fasi = ['TAKE-OFF', 'EXPLORATION', 'FULL VISIT', 'RETURN']
        colori = [(120, 140, 170), (66, 133, 244), TEAL, ARANCIO]
        f_att = self.fase(t)
        x = PX + 18
        for n, nome in enumerate(fasi):
            if n == 2 and 'fine_visita' not in self.ev:
                continue   # volo senza fase di visita
            w = dr.textlength(nome, font=self.f_chip) + 26
            if n == f_att:
                dr.rounded_rectangle([x, PY + 16, x + w, PY + 46], radius=15, fill=colori[n])
                dr.text((x + 13, PY + 23), nome, font=self.f_chip, fill=(255, 255, 255))
            else:
                dr.rounded_rectangle([x, PY + 16, x + w, PY + 46], radius=15, outline=(70, 84, 104), width=2)
                dr.text((x + 13, PY + 23), nome, font=self.f_chip, fill=(110, 124, 146))
            x += w + 10
        mappata = 100.0 * np.searchsorted(self.visto_area_t, t, side='right') / self.n_area
        valori = [('MISSION TIME', mmss(t - self.ev['decollo'])),
                  ('SPEED', f'{max(0.0, sv):.2f} m/s'),
                  ('DISTANCE FLOWN', f'{sdist:.0f} m'),
                  ('AREA MAPPED BY THE LIDAR', f'{mappata:.1f} %')]
        for n, (lab, val) in enumerate(valori):
            gx_ = PX + 24 + (n % 2) * (PW // 2)
            gy_ = PY + 66 + (n // 2) * 92
            dr.text((gx_, gy_), lab, font=self.f_lab, fill=GRIGIO)
            dr.text((gx_, gy_ + 22), val, font=self.f_num, fill=(245, 248, 252))
        dr.text((PX + 24, PY + PH - 30), 'Onboard localization only: 3D LiDAR + IMU, no GPS',
                font=self.f_small, fill=GRIGIO)
        # --- intestazione
        dr.text((40, 18), self.titolo, font=self.f_tit, fill=(245, 248, 252))
        dr.text((40, 64), self.sottotitolo + f'  ·  replay {self.vel:g}x', font=self.f_sub, fill=GRIGIO)
        return img


def schermata(righe, sotto=''):
    """Schermata di testo centrata: righe = [(testo, dimensione, grassetto, colore)]."""
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    alt = sum(r[1] * 1.5 for r in righe)
    y = (H - alt) / 2
    for testo, dim, gras, col in righe:
        f = font(dim, gras)
        w = dr.textlength(testo, font=f)
        dr.text(((W - w) / 2, y), testo, font=f, fill=col)
        y += dim * 1.5
    if sotto:
        f = font(18)
        w = dr.textlength(sotto, font=f)
        dr.text(((W - w) / 2, H - 60), sotto, font=f, fill=GRIGIO)
    return img


def risultati(titolo, voci, nota):
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    f_t, f_n, f_d = font(44, True), font(40, True), font(22)
    dr.text((140, 110), titolo, font=f_t, fill=(245, 248, 252))
    y = 230
    for numero, desc in voci:
        dr.rounded_rectangle([140, y, 1780, y + 118], radius=16, fill=(20, 28, 40))
        dr.text((180, y + 18), numero, font=f_n, fill=ARANCIO)
        dr.text((180, y + 72), desc, font=f_d, fill=(205, 214, 228))
        y += 138
    w = dr.textlength(nota, font=font(18))
    dr.text(((W - w) / 2, H - 60), nota, font=font(18), fill=GRIGIO)
    return img


def orbita_3d(ply, volo_xyz, cartella_frame, n_frame, titolo, sottotitolo, start_idx):
    """Mappa 3D in orbita (Open3D, fuori schermo), con la traiettoria."""
    import open3d as o3d
    from PIL import Image, ImageDraw
    mesh = o3d.io.read_triangle_mesh(ply)
    v = np.asarray(mesh.vertices)
    tetto = mesh.get_axis_aligned_bounding_box().max_bound[2]
    mesh.remove_vertices_by_mask(v[:, 2] > tetto - 0.35)
    mesh.compute_vertex_normals()
    mesh = riduci_per_render(mesh)
    r = o3d.visualization.rendering.OffscreenRenderer(RENDER_W, RENDER_H)
    r.scene.set_background([BG[0] / 255, BG[1] / 255, BG[2] / 255, 1.0])
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = 'defaultLit'
    r.scene.add_geometry('mappa', mesh, mat)
    pts = volo_xyz
    linee = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts),
                                 o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(pts) - 1)]))
    linee.paint_uniform_color([ARANCIO[0] / 255, ARANCIO[1] / 255, ARANCIO[2] / 255])
    ml = o3d.visualization.rendering.MaterialRecord()
    ml.shader = 'unlitLine'
    ml.line_width = 4.0
    r.scene.add_geometry('volo', linee, ml)
    bb = mesh.get_axis_aligned_bounding_box()
    c = bb.get_center()
    est = bb.get_extent()
    raggio = 1.4 * max(est[0], est[1])
    r.setup_camera(45.0, c, c + np.array([raggio, 0, raggio]), [0, 0, 1])
    r.render_to_image()   # il primo fotogramma esce spesso vuoto
    f_t, f_s = font(36, True), font(20)
    for n in range(n_frame):
        u = n / max(1, n_frame - 1)
        ang = -math.pi / 2 + 2 * math.pi * u * 0.75
        elev = math.radians(62 - 22 * u)
        occhio = c + np.array([raggio * math.cos(elev) * math.cos(ang), raggio * math.cos(elev) * math.sin(ang),
                               raggio * math.sin(elev)])
        r.setup_camera(45.0, c, occhio, [0, 0, 1])
        im = rendi(r)
        dr = ImageDraw.Draw(im, 'RGBA')
        dr.rectangle([0, 0, W, 100], fill=BG + (200,))
        dr.text((40, 18), titolo, font=f_t, fill=(245, 248, 252))
        dr.text((40, 64), sottotitolo, font=f_s, fill=GRIGIO)
        im.save(os.path.join(cartella_frame, f'f{start_idx + n:05d}.jpg'), quality=92)
    return n_frame


def codifica(cartella_frame, uscita, fps=None):
    fps = fps or FPS
    bitrate = 9000 if fps <= 30 else 16000
    cmd = ['gst-launch-1.0', '-e', '-q', 'multifilesrc', f'location={cartella_frame}/f%05d.jpg', 'index=0',
           f'caps=image/jpeg,framerate={fps}/1', '!', 'jpegdec', '!', 'videoconvert', '!',
           'video/x-raw,format=I420', '!', 'x264enc', f'bitrate={bitrate}', 'speed-preset=medium',
           f'key-int-max={2 * fps}',
           '!', 'video/x-h264,profile=high', '!', 'h264parse', '!', 'mp4mux', '!', 'filesink',
           f'location={uscita}']
    subprocess.run(cmd, check=True)


def main():
    from PIL import Image
    ap = argparse.ArgumentParser()
    ap.add_argument('cartella')
    ap.add_argument('run', type=int)
    ap.add_argument('uscita')
    ap.add_argument('--mondo', default='cubicle_office')
    ap.add_argument('--spawn', default='-13.5,-4')
    ap.add_argument('--velocita', type=float, default=5.0)
    ap.add_argument('--titolo', default='Autonomous drone: indoor exploration and 3D mapping, no GPS')
    ap.add_argument('--sottotitolo', default='SIMULATION  ·  Gazebo + PX4  ·  31 x 31 m office  ·  flight at 1 m/s')
    ap.add_argument('--risultati', default='', help='file JSON: {"titolo":..., "voci":[[numero, descrizione],...], "nota":...}')
    ap.add_argument('--frame', default='', help='cartella temporanea dei fotogrammi')
    ap.add_argument('--rifai', action='store_true')
    ap.add_argument('--fps', type=int, default=60)
    a = ap.parse_args()
    global FPS
    FPS = a.fps
    d = prepara(a)
    cartella = a.frame or os.path.splitext(a.uscita)[0] + '_frame'
    shutil.rmtree(cartella, ignore_errors=True)
    os.makedirs(cartella)
    n = 0

    def salva(im, ripeti=1):
        nonlocal n
        for _ in range(ripeti):
            im.save(os.path.join(cartella, f'f{n:05d}.jpg'), quality=92)
            n += 1

    def dissolvi(a_img, b_img, secondi=0.6):
        k = int(secondi * FPS)
        for i in range(1, k + 1):
            salva(Image.blend(a_img, b_img, i / (k + 1)))

    nero = Image.new('RGB', (W, H), BG)
    # 1. titolo: il titolo grande, la riga sotto e la nota "Simulation" ci sono dal
    # primo fotogramma (i lettori e le slide lo usano come copertina: con la
    # dissolvenza dal nero la copertina era nera); le due righe di descrizione
    # entrano in dissolvenza.
    descrizione = [('The drone explores a 31 x 31 m office on its own' +
                    (', visits all of it,' if 'fine_visita' in d['eventi'] else ','), 26, False, GRIGIO),
                   ('builds the 3D map and returns to land at its take-off point.', 26, False, GRIGIO)]
    testa = [('Autonomous exploration and 3D mapping', 64, True, (245, 248, 252)),
             ('single drone  ·  indoor environments  ·  no GPS', 32, False, (205, 214, 228)),
             ('', 20, False, BG)]
    piede = 'Simulation: Gazebo + PX4 SITL, simulated 3D LiDAR and camera'
    copertina = schermata(testa + [(r[0], r[1], r[2], BG) for r in descrizione], piede)
    tit = schermata(testa + descrizione, piede)
    salva(copertina, int(0.8 * FPS))
    dissolvi(copertina, tit, 1.0)
    salva(tit, int(3.8 * FPS))
    # 2. replay
    rp = Replay(d, a.velocita, a.titolo, a.sottotitolo)
    t0, t1 = d['eventi']['decollo'] - 1.0, d['eventi']['appoggio'] + 3.0
    tempi = np.arange(t0, t1, a.velocita / FPS)
    primo = rp.frame(tempi[0])
    dissolvi(tit, primo)
    print(f'[video] replay: {len(tempi)} fotogrammi...', flush=True)
    for i, t in enumerate(tempi):
        salva(rp.frame(t))
        if i % 300 == 0:
            print(f'[video]   {i}/{len(tempi)}', flush=True)
    ultimo = rp.frame(tempi[-1])
    salva(ultimo, int(2.0 * FPS))
    ultimo.save(os.path.splitext(a.uscita)[0] + '_fine_volo.png')
    # 3. mappa 3D
    i = int(np.searchsorted(d['volo_t'], d['eventi']['decollo']))
    j = int(np.searchsorted(d['volo_t'], d['eventi']['appoggio']))
    xyz = np.c_[d['volo_x'][i:j:5], d['volo_y'][i:j:5], d['volo_z'][i:j:5]]
    ply = os.path.join(a.cartella, f'rel_mappa3d_{a.run}_1', 'mappa_realistica.ply')
    print('[video] mappa 3D in orbita...', flush=True)
    inizio_3d = n + int(0.6 * FPS)
    tmp = os.path.join(cartella, '_3d')
    os.makedirs(tmp)
    k3 = orbita_3d(ply, xyz, tmp, int(14 * FPS), '3D map built during the flight',
                   'geometry from the LiDAR  ·  color from the camera  ·  drone flight path in yellow', 0)
    primo3d = Image.open(os.path.join(tmp, 'f00000.jpg'))
    dissolvi(ultimo, primo3d)
    for q in range(k3):
        shutil.move(os.path.join(tmp, f'f{q:05d}.jpg'), os.path.join(cartella, f'f{n:05d}.jpg'))
        n += 1
    os.rmdir(tmp)
    ultimo3d = Image.open(os.path.join(cartella, f'f{n - 1:05d}.jpg'))
    ultimo3d.save(os.path.splitext(a.uscita)[0] + '_mappa3d.png')
    # 4. risultati
    if a.risultati:
        r = json.load(open(a.risultati))
        fin = risultati(r['titolo'], r['voci'], r['nota'])
        dissolvi(ultimo3d, fin)
        salva(fin, int(7.0 * FPS))
        fin.save(os.path.splitext(a.uscita)[0] + '_risultati.png')
        dissolvi(fin, nero)
    else:
        dissolvi(ultimo3d, nero)
    print(f'[video] codifica di {n} fotogrammi ({n / FPS:.0f} s)...', flush=True)
    codifica(cartella, a.uscita)
    shutil.rmtree(cartella, ignore_errors=True)
    print(f'[video] pronto: {a.uscita}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
