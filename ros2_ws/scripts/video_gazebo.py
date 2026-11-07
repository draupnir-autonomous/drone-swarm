#!/usr/bin/env python3
"""Montaggio della registrazione della GUI di Gazebo (registra_demo_gazebo.sh).

Accelera la registrazione (prende un fotogramma ogni N), toglie la barra del
titolo della finestra e aggiunge le scritte: SIMULAZIONE, tempo di missione,
fase del volo, fattore di accelerazione. Tempi e fasi dal log della run (stessi
orologi: i log ROS e la registrazione usano l'ora di sistema).

Uso: video_gazebo.py <gazebo_grezzo.mp4> <log_esplorazione> <uscita.mp4> [--velocita 10]
     [--inizio_registrazione <epoca s>]   (default: data di creazione del file)
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
from video_demo import font, mmss, codifica, BG, ARANCIO, TEAL, GRIGIO, W, H, FPS  # noqa: E402

FPS_REGISTRAZIONE = 60   # registra_demo_gazebo.sh registra a 60 fps


def decodifica(mp4, cartella, ogni):
    """Fotogrammi JPEG, uno ogni 'ogni' della registrazione."""
    os.makedirs(cartella, exist_ok=True)
    fps_out = f'{FPS_REGISTRAZIONE}/{ogni}'
    cmd = ['gst-launch-1.0', '-q', 'filesrc', f'location={mp4}', '!', 'qtdemux', '!', 'h264parse', '!',
           'avdec_h264', '!', 'videorate', '!', f'video/x-raw,framerate={fps_out}', '!', 'videoconvert', '!',
           'jpegenc', 'quality=95', '!', 'multifilesink', f'location={cartella}/g%05d.jpg']
    subprocess.run(cmd, check=True)
    return sorted(f for f in os.listdir(cartella) if f.startswith('g'))


def eventi(log):
    chiavi = {'via': r'giro iniziale concluso', 'fine_esplorazione': r'has finished exploration',
              'fine_visita': r'fase conclusa', 'appoggio': r'\[Casa\] sopra il punto di decollo',
              'decollo': r'\[Giro\] quota .* raggiunta'}
    out = {}
    for l in open(log, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\]', l)
        if not m:
            continue
        for k, rx in chiavi.items():
            if k not in out and re.search(rx, l):
                out[k] = float(m.group(1))
    return out


def main():
    from PIL import Image, ImageDraw
    ap = argparse.ArgumentParser()
    ap.add_argument('grezzo')
    ap.add_argument('log')
    ap.add_argument('uscita')
    ap.add_argument('--velocita', type=int, default=5)
    ap.add_argument('--inizio_registrazione', type=float, default=0.0)
    ap.add_argument('--taglio_alto', type=int, default=-1, help='px da togliere in alto (barra del titolo)')
    ap.add_argument('--cambio', type=float, default=0.0,
                    help='ora (epoca) prima della quale accelerare di piu\' (es. vista d\'insieme)')
    ap.add_argument('--velocita_prima', type=int, default=30)
    a = ap.parse_args()
    ev = eventi(a.log)
    t0_reg = a.inizio_registrazione or os.stat(a.grezzo).st_ctime  # sovrascritto sotto se c'e' la nascita
    try:
        nascita = float(subprocess.run(['stat', '-c', '%W', a.grezzo], capture_output=True, text=True).stdout)
        if nascita > 0 and not a.inizio_registrazione:
            t0_reg = nascita
    except ValueError:
        pass
    ogni = max(1, round(FPS_REGISTRAZIONE * a.velocita / FPS))
    cartella = os.path.splitext(a.uscita)[0] + '_frame'
    shutil.rmtree(cartella, ignore_errors=True)
    print('[gazebo] decodifica...', flush=True)
    grezzi = decodifica(a.grezzo, os.path.join(cartella, 'grezzi'), ogni)
    print(f'[gazebo] {len(grezzi)} fotogrammi (uno ogni {ogni})', flush=True)
    dt = ogni / FPS_REGISTRAZIONE
    # barra del titolo: la prima riga arancione dall'alto (colore della barra Material)
    prova = np.asarray(Image.open(os.path.join(cartella, 'grezzi', grezzi[len(grezzi) // 2])).convert('RGB'))
    if a.taglio_alto < 0:
        arancio = (prova[:, :, 0] > 200) & (prova[:, :, 1] > 60) & (prova[:, :, 1] < 120) & (prova[:, :, 2] < 60)
        righe = np.nonzero(arancio.mean(1) > 0.5)[0]
        taglio = int(righe.max()) + 2 if len(righe) else 0
    else:
        taglio = a.taglio_alto
    basso = prova.shape[0]
    while basso > taglio + 10 and prova[basso - 1].mean() < 30:   # bordo nero in basso (~16 dopo il JPEG)
        basso -= 1
    print(f'[gazebo] ritaglio righe {taglio}-{basso}', flush=True)
    f_tit, f_sub, f_lab, f_num, f_chip = font(30, True), font(19), font(15, True), font(34, True), font(15, True)
    t_decollo = ev.get('decollo', t0_reg)
    n = 0
    os.makedirs(os.path.join(cartella, 'out'))
    for k, nome in enumerate(grezzi):
        t = t0_reg + k * dt
        if t < t_decollo - 3:
            continue
        # prima del cambio si accelera di piu': si tiene un fotogramma ogni r
        if a.cambio and t < a.cambio and k % max(1, round(a.velocita_prima / a.velocita)) != 0:
            continue
        fattore = a.velocita_prima if (a.cambio and t < a.cambio) else a.velocita
        im = Image.open(os.path.join(cartella, 'grezzi', nome)).convert('RGB')
        im = im.crop((0, taglio, im.width, basso)).resize((W, H))
        dr = ImageDraw.Draw(im, 'RGBA')
        dr.rectangle([0, 0, W, 92], fill=BG + (215,))
        dr.text((32, 14), 'Volo autonomo nel simulatore', font=f_tit, fill=(245, 248, 252))
        dr.text((32, 56), f'SIMULAZIONE  ·  Gazebo + PX4  ·  vista del simulatore  ·  accelerato {fattore}x',
                font=f_sub, fill=GRIGIO)
        # tempo e fase in basso
        if t < ev.get('via', 1e18):
            fase, col = 'DECOLLO E GIRO INIZIALE', (120, 140, 170)
        elif t < ev.get('fine_esplorazione', 1e18):
            fase, col = 'ESPLORAZIONE', (66, 133, 244)
        elif 'fine_visita' in ev and t < ev['fine_visita']:
            fase, col = 'VISITA COMPLETA', TEAL
        elif t < ev.get('appoggio', 1e18) + 2:
            fase, col = 'RIENTRO', ARANCIO
        else:
            fase, col = 'ATTERRATO', (60, 160, 90)
        dr.rounded_rectangle([24, H - 112, 520, H - 24], radius=14, fill=BG + (215,))
        dr.text((44, H - 100), 'TEMPO DI MISSIONE', font=f_lab, fill=GRIGIO)
        dr.text((44, H - 78), mmss(t - t_decollo), font=f_num, fill=(245, 248, 252))
        w = dr.textlength(fase, font=f_chip) + 28
        dr.rounded_rectangle([250, H - 84, 250 + w, H - 50], radius=16, fill=col)
        dr.text((264, H - 76), fase, font=f_chip, fill=(255, 255, 255))
        im.save(os.path.join(cartella, 'out', f'f{n:05d}.jpg'), quality=92)
        n += 1
        if t > ev.get('appoggio', 1e18) + 6:
            break
    print(f'[gazebo] codifica di {n} fotogrammi ({n / FPS:.0f} s)...', flush=True)
    codifica(os.path.join(cartella, 'out'), a.uscita, FPS)
    shutil.rmtree(cartella, ignore_errors=True)
    print(f'[gazebo] pronto: {a.uscita}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
