#!/usr/bin/env python3
"""PAGELLA DI RUN A UNO O PIU' DRONI, confrontabili riga per riga (sciame, ottobre 2026).

analisi_campagna.py e' pensata per il drone singolo: legge il registro di volo del solo
drone 1 e la sua mappa 3D. Qui ogni run archiviata da run_multi_exploration.sh (un drone
o uno sciame) diventa UNA riga con le stesse colonne:

  droni      quanti droni hanno volato
  esito      finita (tutti hanno concluso l'esplorazione) / TETTO / CRASH / ...
  atterr.    droni rientrati e appoggiati
  miss.s     durata della MISSIONE: dal primo distacco da terra all'ultimo appoggio, in
             secondi SIMULATI (registri di volo di PX4, stesso orologio per tutti i droni):
             e' il tempo che avrebbe il volo vero, qualunque sia il RTF della simulazione
  volo.s     durata del volo piu' lungo fra i droni (decollo-appoggio del singolo drone)
  RTF        fattore di tempo reale medio durante il volo; parete = durata della run
  marg.m     distanza minima VERA dagli ostacoli, sul peggiore dei droni
  fra.m      distanza minima VERA fra due droni in volo (solo sciame)
  incl       inclinazione massima, sul peggiore dei droni
  visit%     spazio percorribile visitato (entro 2 m, con la vista libera) dall'UNIONE dei
             voli, dalla posizione vera (vedi copertura_visita.py)
  cop3d      copertura delle superfici di parete visibili nella mappa 3D finale (fusa,
             per lo sciame)
  pre3d      punti di parete della mappa 3D entro 15 cm da una parete vera
  foto%      quota di pavimento e pareti (sotto 2 m) colorata dalle foto nella mappa 3D
  pav / pend quota (vero 0) e pendenza del pavimento della mappa 3D

Una run e' PULITA se tutti i droni hanno concluso e sono atterrati, nessuno ha toccato un
ostacolo (margine >= raggio del drone), nessuno ha superato 45 gradi e la mappa 3D copre
almeno il 95% delle pareti visibili: la stessa regola di analisi_campagna.py, estesa a tutti
i droni.

Uso: pagella_sciame.py <cartella_run> [<cartella_run> ...]
"""
import glob
import json
import os
import re
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import analisi_campagna as ac  # noqa: E402
import clearance_report as cr  # noqa: E402
import copertura_visita as cv  # noqa: E402

try:
    from pyulog import ULog
except ImportError:
    ULog = None

COPERTURA_MIN = 95.0


def mondo_della_run(cartella, info):
    """Stessa scelta di analisi_campagna.py: per i mondi del progetto l'originale (che ha
    accanto la pianta degli ostacoli), altrimenti la copia archiviata."""
    mondo = os.path.join(cartella, 'rel_mondo.sdf')
    if info.get('origine') and os.path.exists(info['origine']):
        orig = info['origine']
        if os.path.exists(os.path.splitext(orig)[0] + '_ostacoli.npz') or info.get('tarato'):
            mondo = orig
    if info.get('origine'):
        ac.RADICI_EXTRA.append(os.path.dirname(info['origine']))
    return mondo


def verita(ulg):
    """Tempo [s] e posizione VERA (mondo: x, y, quota sul terreno) di un registro di volo."""
    if ULog is None or not os.path.exists(ulg):
        return None
    try:
        g = ULog(ulg, ['vehicle_local_position_groundtruth']).data_list[0].data
    except Exception:
        return None
    return (g['timestamp'] / 1e6, np.asarray(g['y']), np.asarray(g['x']),
            -np.asarray(g['z']) + ac.QUOTA_APPOGGIO)


def distanza_fra_droni(veri):
    """Minima distanza 3D fra due droni entrambi in volo (quota > 0.4 m), su una griglia di
    tempo comune a 20 Hz. I registri di PX4 condividono l'orologio della simulazione."""
    minimo = None
    for a in range(len(veri)):
        for b in range(a + 1, len(veri)):
            if veri[a] is None or veri[b] is None:
                continue
            ta, tb = veri[a][0], veri[b][0]
            t0, t1 = max(ta[0], tb[0]), min(ta[-1], tb[-1])
            if t1 <= t0:
                continue
            t = np.arange(t0, t1, 0.05)
            pa = np.stack([np.interp(t, ta, veri[a][k]) for k in (1, 2, 3)], axis=1)
            pb = np.stack([np.interp(t, tb, veri[b][k]) for k in (1, 2, 3)], axis=1)
            in_volo = (pa[:, 2] > 0.4 + ac.QUOTA_APPOGGIO) & (pb[:, 2] > 0.4 + ac.QUOTA_APPOGGIO)
            if not in_volo.any():
                continue
            d = float(np.linalg.norm(pa[in_volo] - pb[in_volo], axis=1).min())
            minimo = d if minimo is None else min(minimo, d)
    return minimo


def pianta_visita(mondo):
    """(occupato, x0, y0) alla quota di volo: box del mondo, o la pianta del generatore
    (parco_ostacoli.npz) per i mondi fatti di modelli."""
    npz = os.path.splitext(os.path.realpath(mondo))[0] + '_ostacoli.npz'
    if os.path.exists(npz):
        z = np.load(npz)
        if abs(float(z['res']) - cv.RES) < 1e-9:
            return z['dist'] <= 0.0, float(z['x0']), float(z['y0'])
    if cr.boxes_from_sdf(mondo, 0.4, 1.6):
        return cv.pianta(mondo)
    return None


def spazio_visitato(mondo, veri, partenza):
    p = pianta_visita(mondo)
    if p is None:
        return None
    occ, x0, y0 = p
    dist = cr.edt(occ) * cv.RES
    area = cv.raggiungibili(dist >= 0.5, int((partenza[0] - x0) / cv.RES),
                            int((partenza[1] - y0) / cv.RES))
    vis = np.zeros_like(occ)
    for v in veri:
        if v is None:
            continue
        m = v[3] > 0.5 + ac.QUOTA_APPOGGIO
        vis |= cv.visita(occ, x0, y0, list(zip(v[1][m], v[2][m])), 2.0)
    return 100.0 * np.count_nonzero(vis & area) / max(1, np.count_nonzero(area))


def leggi_testo(p):
    return open(p, errors='replace').read() if os.path.exists(p) else ''


def analizza(cartella):
    info = {}
    if os.path.exists(os.path.join(cartella, 'rel_mondo.json')):
        info = json.load(open(os.path.join(cartella, 'rel_mondo.json')))
    mondo = mondo_della_run(cartella, info)
    runtxt = leggi_testo(os.path.join(cartella, 'run.txt'))
    run = leggi_testo(os.path.join(cartella, 'rel_run_1.log'))
    expl = leggi_testo(os.path.join(cartella, 'rel_expl_1.log'))
    ulogs = sorted(glob.glob(os.path.join(cartella, 'rel_ulog_1_*.ulg')),
                   key=lambda p: int(re.search(r'_(\d+)\.ulg$', p).group(1)))
    m = re.search(r'^droni:\s+(\d+)', runtxt, re.M)
    droni = int(m.group(1)) if m else max(1, len(ulogs))
    partenze = [[float(v) for v in s.split(',')[:2]] for s in info.get('partenze', [])]
    while len(partenze) < droni:
        partenze.append([0.0, 0.0])

    r = {'run': os.path.basename(os.path.normpath(cartella)), 'droni': droni,
         'mondo': info.get('nome') or os.path.splitext(os.path.basename(mondo))[0]}
    finiti = len(set(re.findall(r'Drone (\d+) has finished exploration', expl)))
    if re.search(r'CRASH DETECTED', expl):
        r['esito'] = 'CRASH'
    elif re.search(r'DISARMATO IN VOLO|VOLO INTERROTTO', expl):
        r['esito'] = 'DISARMO'
    elif re.search(r'TEMPO MASSIMO', expl):
        r['esito'] = 'TETTO'
    elif finiti >= droni:
        r['esito'] = 'finita'
    elif 'Timeout raggiunto' in run:
        r['esito'] = 'TETTO'
    else:
        r['esito'] = 'INTERROTTA'
    r['atterrati'] = len(set(re.findall(r'\[Rientro\] Drone (\d+) (?:arrivato|gia\' alla partenza|tempo scaduto)',
                                        expl)))
    m = re.search(r'RTF medio:\s+([\d.]+)', runtxt)
    r['rtf'] = float(m.group(1)) if m else None
    t0 = re.search(r'^inizio:\s+(.+)$', runtxt, re.M)
    t1 = re.search(r'^fine:\s+(\S+ \S+)', runtxt, re.M)
    if t0 and t1:
        from datetime import datetime
        f = '%Y-%m-%d %H:%M:%S'
        r['parete_s'] = (datetime.strptime(t1.group(1), f) - datetime.strptime(t0.group(1), f)).seconds

    dist_info = ac.griglia_distanze(mondo)
    dist3d = ac.distanze_3d(mondo)
    per_drone, veri = [], []
    for i, u in enumerate(ulogs):
        d = ac.analizza_ulog(u, partenze[i], dist_info, dist3d)
        v = verita(u)
        if v is not None:
            t, _, _, alt = v
            # Decollo = prima volta sopra 0.3 m per almeno 2 s di fila. Il primo campione
            # del registro (che parte all'avvio di PX4) puo' stare a 0.21 m mentre il modello
            # si assesta allo spawn: con "sopra 0.2 m" la missione del drone 1 partiva
            # dall'avvio di PX4, ~40 s prima del decollo vero (prima run a tre droni).
            sopra = alt - ac.QUOTA_APPOGGIO > 0.3
            decollo = None
            for i in np.where(sopra)[0]:
                fine = np.searchsorted(t, t[i] + 2.0)
                if fine < len(t) and sopra[i:fine].all():
                    decollo = i
                    break
            staccato = np.where(alt - ac.QUOTA_APPOGGIO > 0.2)[0]
            if decollo is not None and staccato.size:
                d['decollo_t'], d['appoggio_t'] = float(t[decollo]), float(t[staccato[-1]])
                d['missione_s'] = d['appoggio_t'] - d['decollo_t']
        per_drone.append(d)
        veri.append(v)
    r['per_drone'] = per_drone
    decolli = [d['decollo_t'] for d in per_drone if 'decollo_t' in d]
    appoggi = [d['appoggio_t'] for d in per_drone if 'appoggio_t' in d]
    if decolli and appoggi:
        r['missione_s'] = max(appoggi) - min(decolli)
    voli = [d['missione_s'] for d in per_drone if d.get('missione_s')]
    r['volo_max_s'] = max(voli) if voli else None

    def peggiore(chiave, f):
        vals = [d[chiave] for d in per_drone if d.get(chiave) is not None]
        return f(vals) if vals else None
    r['margine_min'] = peggiore('margine_min', min)
    r['sotto_raggio_pct'] = peggiore('sotto_raggio_pct', max)
    r['incl_max'] = peggiore('incl_max', max)
    r['quota_min'] = peggiore('quota_min', min)
    r['vel_media'] = peggiore('vel_media', np.mean)
    r['err_stima_med'] = peggiore('err_stima_med', max)
    r['fra_droni'] = distanza_fra_droni(veri) if droni > 1 else None
    try:
        r['visita'] = spazio_visitato(mondo, veri, partenze[0])
    except Exception as e:  # pianta non ricavabile: la colonna resta vuota
        print(f'  ({r["run"]}: spazio visitato non calcolato: {e})')
        r['visita'] = None

    cartella3d = os.path.join(cartella, 'rel_mappa3d_1_sciame')
    if not os.path.isdir(cartella3d):
        cartella3d = os.path.join(cartella, 'rel_mappa3d_1_1')
    ply = os.path.join(cartella3d, 'mappa_realistica.ply')
    campioni = ac.campioni_osservabili(mondo)
    m3d = ac.carica_mesh(ply)
    r['cop3d'] = ac.copertura(m3d, campioni) if m3d is not None else None
    r['prec3d'] = ac.precisione(m3d, mondo) if m3d is not None else None
    r['pav'], r['pend'] = ac.pavimento(m3d)
    r['foto'] = ac.foto_mappa3d(ply)
    pose = os.path.join(cartella3d, 'pose_fotogrammi.json')
    r['fotogrammi'] = len(json.load(open(pose))['fotogrammi']) if os.path.exists(pose) else None
    r['pulita'] = (r['esito'] == 'finita' and r['atterrati'] >= droni
                   and (r['sotto_raggio_pct'] or 0) == 0 and (r['incl_max'] or 0) < 45
                   and (r['cop3d'] is None or r['cop3d'] >= COPERTURA_MIN))
    return r


def f(v, fmt):
    return '-' if v is None else format(v, fmt)


def main():
    cartelle = [c for c in sys.argv[1:] if os.path.isdir(c)]
    if not cartelle:
        print(__doc__)
        return 1
    righe = [analizza(c) for c in cartelle]
    print(f'{"run":<52} {"droni":>5} {"esito":>8} {"att.":>4} {"miss.s":>6} {"volo.s":>6} '
          f'{"RTF":>4} {"parete":>6} {"marg.m":>6} {"fra.m":>5} {"incl":>4} {"q.min":>5} '
          f'{"v.med":>5} {"visit%":>6} {"cop3d":>5} {"pre3d":>5} {"foto%":>5} {"foto":>5} '
          f'{"pav":>6} {"pulita":>6}')
    for r in righe:
        print(f'{r["run"][-52:]:<52} {r["droni"]:>5} {r["esito"]:>8} '
              f'{r["atterrati"]:>2}/{r["droni"]:<1} {f(r.get("missione_s"), ".0f"):>6} '
              f'{f(r.get("volo_max_s"), ".0f"):>6} {f(r.get("rtf"), ".2f"):>4} '
              f'{f(r.get("parete_s"), "d"):>6} {f(r.get("margine_min"), ".2f"):>6} '
              f'{f(r.get("fra_droni"), ".2f"):>5} {f(r.get("incl_max"), ".0f"):>4} '
              f'{f(r.get("quota_min"), ".2f"):>5} {f(r.get("vel_media"), ".2f"):>5} '
              f'{f(r.get("visita"), ".1f"):>6} {f(r.get("cop3d"), ".1f"):>5} '
              f'{f(r.get("prec3d"), ".1f"):>5} {f(r.get("foto"), ".1f"):>5} '
              f'{f(r.get("fotogrammi"), "d"):>5} {f(r.get("pav"), "+.2f"):>6} '
              f'{("si" if r["pulita"] else "NO"):>6}')
        for i, d in enumerate(r['per_drone']):
            print(f'    drone {i + 1}: volo {f(d.get("missione_s"), ".0f")} s, margine '
                  f'{f(d.get("margine_min"), ".2f")} m, inclinazione {f(d.get("incl_max"), ".0f")}, '
                  f'quota min {f(d.get("quota_min"), ".2f")} m, vel. media '
                  f'{f(d.get("vel_media"), ".2f")} m/s, errore stima {f(d.get("err_stima_med"), ".2f")} m, '
                  f'appoggio a {f(d.get("dist_casa"), ".2f")} m dal decollo')
    print()
    print('miss.s = dal primo decollo all\'ultimo appoggio, in secondi SIMULATI (il tempo del volo '
          'vero). parete = durata della run sul Jetson. Pulita = tutti conclusi e atterrati, nessun '
          f'contatto, inclinazione < 45, copertura 3D >= {COPERTURA_MIN:.0f}%.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
