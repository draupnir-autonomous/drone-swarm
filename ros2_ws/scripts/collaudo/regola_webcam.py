#!/usr/bin/env python3
"""Esposizione della webcam per le foto della mappa 3D all'aperto (C2-Explorer, novembre 2026).

cam2image (image_tools) apre la webcam senza toccarne l'esposizione. L'8 novembre 2026, al sole,
le foto della missione erano bianche al 75-93% e la mappa 3D e' uscita colorata al 5%. Prima che
la missione avvii cam2image, questo programma porta la webcam in esposizione MANUALE e cerca il
tempo di esposizione che lascia saturo meno di --saturi dei pixel, col drone fermo nel punto di
decollo (inquadratura del campo). I controlli UVC restano impostati nella webcam finche' non la
si scollega, quindi valgono anche per il cam2image della missione.

Se la webcam non accetta i comandi (o OpenCV non riesce ad aprirla) lo dice e lascia tutto com'e':
la missione vola lo stesso, con l'esposizione automatica. Lo chiama la fase 10 del collaudo, a
stima del collaudo ferma (la webcam libera) e prima della missione.

Uso: regola_webcam.py /dev/video0 [--saturi 0.02] [--solo-leggi] [--automatica]
     --automatica rimette l'esposizione automatica (per tornare al chiuso).
Esce 0 anche quando non puo' regolare (2 solo se il dispositivo non esiste); l'ultima riga riassume.
"""
import argparse
import os
import sys

# OpenCV con il backend V4L2: CAP_PROP_AUTO_EXPOSURE vale 1 = manuale e 3 = automatica (priorita'
# di apertura, V4L2_EXPOSURE_APERTURE_PRIORITY); CAP_PROP_EXPOSURE e' exposure_absolute delle webcam
# UVC, in unita' di 100 microsecondi.
MANUALE, AUTOMATICA = 1, 3


def misura(cap, scarta=4, prendi=4):
    """(frazione di pixel saturi, luminosita' media) su alcuni fotogrammi, scartando i primi dopo
    un cambio (la webcam applica il nuovo valore con qualche fotogramma di ritardo)."""
    for _ in range(scarta):
        cap.read()
    sat, lum = [], []
    for _ in range(prendi):
        ok, f = cap.read()
        if not ok or f is None:
            continue
        sat.append(float((f >= 250).any(axis=2).mean()))
        lum.append(float(f.mean()))
    if not sat:
        return None, None
    return sorted(sat)[len(sat) // 2], sorted(lum)[len(lum) // 2]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dev')
    ap.add_argument('--saturi', type=float, default=0.02, help='frazione massima di pixel saturi')
    ap.add_argument('--luminosita-min', type=float, default=50.0, help='media minima (0-255)')
    ap.add_argument('--solo-leggi', action='store_true')
    ap.add_argument('--automatica', action='store_true')
    a = ap.parse_args()
    if not os.path.exists(a.dev):
        print(f'[webcam] {a.dev} non esiste')
        return 2
    import cv2
    num = int(''.join(c for c in a.dev if c.isdigit()) or 0)
    cap = cv2.VideoCapture(num, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[webcam] {a.dev} non si apre (occupata da un altro programma?): esposizione lasciata com\'e\'')
        return 0
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        sat, lum = misura(cap)
        prima = (cap.get(cv2.CAP_PROP_AUTO_EXPOSURE), cap.get(cv2.CAP_PROP_EXPOSURE))
        print(f'[webcam] adesso: modo esposizione {prima[0]:g}, esposizione {prima[1]:g}, '
              f'saturi {100 * (sat or 0):.0f}%, luminosita\' media {lum or 0:.0f}')
        if a.solo_leggi:
            return 0
        if a.automatica:
            ok = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTOMATICA)
            print('[webcam] esposizione automatica ' + ('rimessa' if ok else 'NON accettata'))
            return 0
        if sat is None:
            print('[webcam] nessun fotogramma: esposizione lasciata com\'e\'')
            return 0
        if sat <= a.saturi:
            print(f'[webcam] gia\' a posto (saturi {100 * sat:.1f}%): nessun cambio')
            return 0
        if not cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, MANUALE):
            print('[webcam] la webcam non accetta l\'esposizione manuale: resta automatica')
            return 0
        esp = cap.get(cv2.CAP_PROP_EXPOSURE)
        if not esp or esp <= 0:
            esp = 300.0                       # 30 ms, un punto di partenza tipico al chiuso
        # Dimezza finche' i saturi non scendono sotto la soglia (o la webcam non va piu' giu').
        migliore = None
        for _ in range(12):
            if not cap.set(cv2.CAP_PROP_EXPOSURE, esp):
                print(f'[webcam] esposizione {esp:g} non accettata')
                break
            sat, lum = misura(cap)
            if sat is None:
                break
            print(f'[webcam]   esposizione {esp:g} ({esp / 10:.1f} ms): saturi {100 * sat:.1f}%, '
                  f'luminosita\' {lum:.0f}')
            if sat <= a.saturi:
                migliore = (esp, sat, lum)
                break
            if esp <= 1:
                migliore = (esp, sat, lum)
                break
            esp = max(1.0, esp / 2)
        if migliore is None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTOMATICA)
            print('[webcam] regolazione non riuscita: rimessa l\'esposizione automatica')
            return 0
        esp, sat, lum = migliore
        if lum < a.luminosita_min:
            print(f'[webcam] ATTENZIONE: immagine scura (luminosita\' {lum:.0f}) con esposizione {esp:g}')
        print(f'[webcam] esposizione MANUALE {esp:g} ({esp / 10:.1f} ms): saturi {100 * sat:.1f}%, '
              f'luminosita\' {lum:.0f} (per tornare all\'automatica: regola_webcam.py {a.dev} --automatica)')
        return 0
    finally:
        cap.release()


if __name__ == '__main__':
    sys.exit(main())
