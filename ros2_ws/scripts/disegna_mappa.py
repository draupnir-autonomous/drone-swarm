#!/usr/bin/env python3
"""Immagine della mappa esplorata, confrontata con la pianta VERA del mondo.

Sostituisce il grafico che lo script di run produceva a fine missione: quello
disegnava dall'alto TUTTI i punti della mappa, pavimento e soffitto compresi, e
il risultato era una macchia piena che non somigliava a nessuna pianta - non
permetteva di giudicare la mappa, ne' bene ne' male.

Qui si mostrano solo le pareti (punti fra 0.3 e 2.0 m di quota):
  sinistra  mappa con sopra il contorno delle pareti vere del mondo .sdf
  destra    gli stessi punti colorati per errore: verde se cadono entro 15 cm da
            una parete vera, rosso se no; nel titolo precisione (quota di punti
            giusti) e copertura (quota di pareti vere viste)

Uso: disegna_mappa.py <uscita.png> <mondo.sdf> <config.yaml> <mappa.pcd> [...]
"""
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analisi_campagna as ac  # noqa: E402

Z_LO, Z_HI = 0.3, 2.0
TOLL = 0.15


def box_mappa(config):
    """sdf_map.box_min/max x,y dal file di configurazione dello scenario."""
    testo = open(config).read()
    v = {}
    for k in ('box_min_x', 'box_min_y', 'box_max_x', 'box_max_y'):
        m = re.search(r'sdf_map\.' + k + r':\s*(-?[\d.]+)', testo)
        v[k] = float(m.group(1)) if m else None
    if None in v.values():
        return None
    return ((v['box_min_x'], v['box_min_y']), (v['box_max_x'], v['box_max_y']))


def distanza_pareti(px, py, pareti):
    d = np.full(len(px), np.inf)
    for cx, cy, _cz, hx, hy, _hz in pareti:
        dx = np.maximum(np.abs(px - cx) - hx, 0)
        dy = np.maximum(np.abs(py - cy) - hy, 0)
        d = np.minimum(d, np.sqrt(dx * dx + dy * dy))
    return d


def main():
    if len(sys.argv) < 5:
        print(__doc__)
        return 1
    uscita, mondo, config, pcds = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
    pareti = [b for b in ac.box_3d(mondo)
              if b[5] * 2 < 3 and b[2] - b[5] < Z_HI and b[2] + b[5] > Z_LO]
    nuvole = [ac.carica_pcd(p) for p in pcds if os.path.exists(p)]
    if not nuvole:
        print('nessuna mappa da disegnare')
        return 1
    pts = np.vstack(nuvole)
    pts = pts[(pts[:, 2] > Z_LO) & (pts[:, 2] < Z_HI)]

    d = distanza_pareti(pts[:, 0], pts[:, 1], pareti)
    giusti = d < TOLL
    precisione = 100.0 * giusti.mean() if len(pts) else 0.0
    bm = box_mappa(config)
    copertura = None
    if bm is not None:
        campioni = ac.campioni_osservabili(mondo)
        copertura = ac.copertura(pcds[0], campioni) if len(pcds) == 1 else None

    fig, ax = plt.subplots(1, 2, figsize=(16, 8))
    for a in ax:
        for cx, cy, _cz, hx, hy, _hz in pareti:
            a.add_patch(plt.Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy,
                                      fill=False, ec='0.2', lw=0.6))
        a.set_aspect('equal')
        a.set_xlabel('x (m)')
        a.set_ylabel('y (m)')
    ax[0].scatter(pts[:, 0], pts[:, 1], s=0.6, c='tab:blue')
    ax[0].set_title(f'Mappa esplorata (pareti {Z_LO}-{Z_HI} m) e pianta reale - {len(pts)} punti')
    ax[1].scatter(pts[giusti, 0], pts[giusti, 1], s=0.6, c='tab:green', label='su una parete vera')
    ax[1].scatter(pts[~giusti, 0], pts[~giusti, 1], s=1.5, c='tab:red',
                  label=f'oltre {int(TOLL * 100)} cm da ogni parete')
    titolo = f'Errori: precisione {precisione:.0f}%'
    if copertura is not None:
        titolo += f', copertura delle pareti visibili {copertura:.0f}%'
    ax[1].set_title(titolo)
    ax[1].legend(markerscale=8, fontsize=9, loc='upper right')
    plt.tight_layout()
    plt.savefig(uscita, dpi=120)
    msg = f'Salvata in {uscita}: precisione {precisione:.0f}%'
    if copertura is not None:
        msg += f', copertura {copertura:.0f}%'
    print(msg + f' ({len(pts)} punti di parete)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
