#!/usr/bin/env python3
"""Configurazione dello stack di esplorazione per il drone VERO (C2-Explorer, ottobre 2026).

Il gemello di scripts/prepara_mondo.py senza Gazebo: non c'e' un mondo da leggere, c'e' solo
la ZONA DA ESPLORARE (il recinto della missione), che l'operatore misura sul posto. Scrive in
--uscita quello che serve a scripts/run_reale_exploration.sh:

  config.yaml   parametri dello stack: la stessa base delle simulazioni (prepara_mondo.py,
                multi_drone_cubicle_office.yaml) con zona, mappa e tetto al tempo di
                esplorazione di questa missione
  mondo.json    lo stesso in forma leggibile, archiviato con la run

COORDINATE: quelle di FAST-LIO, cioe' del drone al momento dell'avvio della stima, appoggiato
nel punto di decollo: origine nel punto di decollo, x in AVANTI (dove punta il muso), y a
SINISTRA, z in su con 0 = pavimento. Il recinto BOX="xmin,ymin,xmax,ymax" e' in quei metri.

Il punto di decollo deve stare nel recinto con almeno 1 m di margine per lato: il drone sale in
verticale e fa un giro su se stesso prima di partire.

Con --partenza x,y (solo la prova in simulazione, run_reale_exploration.sh sim) il punto di
decollo non e' l'origine del mondo: il recinto resta relativo al decollo e viene spostato li'.

Uso: prepara_reale.py --box -2,-2,3,2 [--quota-zona 2.2] [--tempo-max 240] --uscita <cartella>
"""
import argparse
import json
import os
import re
import sys

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(QUI))
import prepara_mondo as pm  # noqa: E402

MARGINE_DECOLLO = 1.0   # m fra il punto di decollo e ogni lato del recinto
LATO_MIN, LATO_MAX = 2.5, 40.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--box', required=True, help='xmin,ymin,xmax,ymax in metri (vedi COORDINATE)')
    ap.add_argument('--quota-zona', type=float, default=2.2,
                    help='quota massima della zona da esplorare (m da terra); il rientro vola a 1.8 m')
    ap.add_argument('--tempo-max', type=float, default=240.0,
                    help='tetto al tempo di esplorazione (s dal via): poi rientra e atterra')
    ap.add_argument('--partenza', default='0,0', help='x,y del punto di decollo nel mondo (simulazione)')
    ap.add_argument('--uscita', required=True)
    a = ap.parse_args()
    px, py = [float(v) for v in a.partenza.split(',')[:2]]
    try:
        xmin, ymin, xmax, ymax = [float(v) for v in a.box.split(',')]
    except ValueError:
        sys.exit(f'ERRORE: BOX="{a.box}" non e\' "xmin,ymin,xmax,ymax"')
    for lo, hi, ax in ((xmin, xmax, 'x'), (ymin, ymax, 'y')):
        if not LATO_MIN <= hi - lo <= LATO_MAX:
            sys.exit(f'ERRORE: il recinto lungo {ax} e\' {hi - lo:.1f} m (ammessi {LATO_MIN}-{LATO_MAX})')
        if lo > -MARGINE_DECOLLO or hi < MARGINE_DECOLLO:
            sys.exit(f'ERRORE: il punto di decollo (0,0) deve stare nel recinto con almeno '
                     f'{MARGINE_DECOLLO} m di margine: lungo {ax} il recinto va da {lo} a {hi}')
    # Il rientro vola a 1.8 m (return_altitude_ nell'FSM) e il riferimento non supera
    # max_flight_z (2.0 nel launch): la zona deve contenerli entrambi.
    if not 2.0 <= a.quota_zona <= 4.0:
        sys.exit(f'ERRORE: quota della zona {a.quota_zona} m fuori da 2.0-4.0 (il rientro e\' a 1.8 m)')
    if a.tempo_max < 30:
        sys.exit('ERRORE: --tempo-max sotto i 30 s non lascia nemmeno il tempo di partire')

    os.makedirs(a.uscita, exist_ok=True)
    testo = pm.config_generica(xmin + px, ymin + py, xmax + px, ymax + py, a.quota_zona, 'reale',
                               'drone vero')
    testo, n = re.subn(r'^(\s*)fsm\.max_explore_time:.*$', rf'\g<1>fsm.max_explore_time: {a.tempo_max:.1f}',
                       testo, count=1, flags=re.M)
    if n != 1:
        sys.exit(f'ERRORE: fsm.max_explore_time non trovato in {pm.CONFIG_BASE}')
    testo = ('# DRONE VERO (scripts/collaudo/prepara_reale.py): coordinate di FAST-LIO, origine nel punto\n'
             '# di decollo, x avanti, y sinistra. Il campo del Mid-360 vero (perception_utils) lo\n'
             '# imposta il launch con reale:=true.\n') + testo
    cfg = os.path.join(a.uscita, 'config.yaml')
    open(cfg, 'w').write(testo)
    info = {'nome': 'reale', 'zona': [xmin, ymin, xmax, ymax], 'partenza': [px, py],
            'quota_zona': a.quota_zona,
            'tempo_max_esplorazione': a.tempo_max, 'config': cfg, 'config_origine': pm.CONFIG_BASE}
    json.dump(info, open(os.path.join(a.uscita, 'mondo.json'), 'w'), indent=1)
    print(f'recinto x {xmin:.1f}..{xmax:.1f}  y {ymin:.1f}..{ymax:.1f}  quota fino a {a.quota_zona} m, '
          f'esplorazione al massimo {a.tempo_max:.0f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
