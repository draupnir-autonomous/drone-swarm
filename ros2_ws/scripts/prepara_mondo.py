#!/usr/bin/env python3
"""Prepara un mondo Gazebo QUALSIASI per una missione di esplorazione.

Riceve un nome (cercato fra i mondi del progetto e quelli di PX4) o il percorso di un
file .sdf e scrive in --uscita tutto quello che serve a run_multi_exploration.sh:

  <nome>.sdf    il mondo normalizzato: nome = nome del file, plugin del mondo tolti
                (li carica il server.config di PX4, uguale per tutti i mondi), fisica a
                250 Hz, coordinate geografiche e luce se mancano, URI relativi resi
                assoluti; per i mondi APERTI un recinto alto 4 m attorno alla zona da
                esplorare (vedi sotto)
  config.yaml   parametri dello stack di esplorazione
  mondo.env     variabili per lo script di run (nome, percorsi, pose di partenza)
  mondo.json    lo stesso in forma leggibile, archiviato con la run

Mondi TARATI (cubicle_office, office_small, empty_room, open_field, parco): pose di
partenza e configurazione misurate nelle campagne, come prima. Per ogni altro mondo si
ricava tutto dalla geometria di collisione (geometria_mondo.py):

  zona da esplorare  ingombro degli ostacoli fra 0.1 e 2.5 m di quota, +0.5 m, al
                     massimo 40 x 40 m (BOX="xmin,ymin,xmax,ymax" per sceglierla)
  aperto o chiuso    raggi orizzontali da punti liberi: se piu' del 5% non incontra
                     nulla il mondo e' aperto. In un mondo aperto i raggi del lidar che
                     non tornano non liberano spazio, l'esplorazione non si chiude mai
                     (e' il motivo della recinzione di open_field e della siepe del
                     parco): si aggiunge un recinto. RECINTO=0/1 forza la scelta.
  quota della zona   2.5 m se sopra la partenza c'e' un soffitto entro 4 m, 2.2 m senza
                     (come cubicle_office e parco)
  partenza           punto libero (>= 1.5 m dagli ostacoli fra 0.3 e 1.3 m, se non c'e'
                     1.2, 1.0, 0.8; niente sopra sotto 1.9 m) piu' vicino al centro dello
                     spazio libero; i droni
                     successivi a 3 m. SPAWN="x,y[,yaw];x,y[,yaw]" per sceglierle.

Il terreno deve essere a quota 0 (tutto lo stack vola a quote assolute): altrimenti
si esce con un errore invece di partire storti.

Uso: prepara_mondo.py <nome|mondo.sdf> --droni N --uscita <cartella>
"""
import argparse
import copy
import json
import os
import re
import shlex
import sys
import xml.etree.ElementTree as ET

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import geometria_mondo as gm  # noqa: E402

WS = os.path.dirname(QUI)
PROGETTO = os.path.dirname(WS)
MONDI = [os.path.join(WS, 'src', 'c2x_worlds', 'worlds'),
         os.path.join(PROGETTO, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'worlds')]
CONFIG = os.path.join(WS, 'src', 'exploration_manager', 'config')
# Configurazione di partenza per i mondi nuovi: quella con tutte le tarature e le
# protezioni (cubicle_office, Jetson, ottobre 2026); cambiano solo zona e mappa.
CONFIG_BASE = 'multi_drone_cubicle_office.yaml'
QUOTA_APPOGGIO = 0.227   # corpo dell'x500 appoggiato sui pattini (vedi run_multi_exploration.sh)
LATO_MAX = 40.0          # lato massimo della zona da esplorare, in metri
LUCE = """<light type="directional" name="sole">
  <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>
</light>"""
COORDINATE = """<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>47.397971057728974</latitude_deg><longitude_deg>8.546163739800146</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>"""

# Mondi gia' tarati: partenze "x,y" (una per drone, nell'ordine) e configurazione,
# misurate nelle campagne. Erano nello script di run fino a ottobre 2026.
TARATI = {
    # In fila lungo y con 4 m di passo. A 1.5 m i droni si contendevano le stesse
    # frontiere iniziali; ai tre angoli (27-38 m) restavano fuori portata radio e non
    # si coordinavano mai. 4 m: settori di partenza distinti ed estremi a 8 m, dentro
    # r_comm (10 m) per il primo giro di MeetingOpt.
    'empty_room': (['0,-4', '0,0', '0,4', '0,-8', '0,8', '4,0'], 'multi_drone_empty_room.yaml'),
    # Lungo il corridoio ovest (x = -13.5), che sulla geometria ricostruita ha almeno
    # 1 m di margine libero da y=-10 a y=+10: la linea di partenza del paper, con 4 m
    # di passo (stesse ragioni di empty_room).
    'cubicle_office': (['-13.5,-4', '-13.5,0', '-13.5,4', '-13.5,-8', '-13.5,8', '-13.5,2'],
                       'multi_drone_cubicle_office.yaml'),
    # Ufficio ritagliato a 16x16 m: la linea x=-3.5 e' quella col margine minimo piu'
    # alto fra quelle provate (0.8-1.3 m nei tre punti).
    'office_small': (['-3.5,-3', '-3.5,0', '-3.5,3', '-3.5,-5', '-3.5,5', '-2.0,0'],
                     'multi_drone_office_small.yaml'),
    # Campo 20x20 m recintato: 3 m di passo nell'area centrale, libera entro 6 m
    # dall'origine (4 m porterebbero i droni esterni troppo vicini agli ostacoli).
    'open_field': (['0,-3', '0,0', '0,3', '3,-3', '3,0', '3,3'], 'multi_drone_open_field.yaml'),
    # Parco 31x31 m (make_parco.py): in fila lungo y=-5.5 fra marciapiede e prato, 3 m
    # di passo; distanze vere dall'ostacolo piu' vicino 2.5, 3.1, 3.2, 2.7, 2.6, 1.3 m.
    'parco': (['-13,-5.5', '-10,-5.5', '-7,-5.5', '-4,-5.5', '-1,-5.5', '2,-5.5'], 'multi_drone_parco.yaml'),
}


def trova_mondo(arg):
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    nome = arg[:-4] if arg.endswith('.sdf') else arg
    for d in MONDI:
        p = os.path.join(d, nome + '.sdf')
        if os.path.isfile(p):
            return os.path.realpath(p)
    sys.exit(f"ERRORE: mondo '{arg}' non trovato (ne' come file, ne' in {', '.join(MONDI)})")


def nome_pulito(percorso):
    n = re.sub(r'[^A-Za-z0-9_]', '_', os.path.splitext(os.path.basename(percorso))[0])
    return n if n and not n[0].isdigit() else 'mondo_' + n


def leggi(percorso):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(percorso, parser)


def normalizza(albero, nome, cartella_origine, avvisi):
    sdf = albero.getroot()
    w = sdf.find('world')
    if w is None:
        sys.exit('ERRORE: nel file non c\'e\' un <world>')
    w.set('name', nome)
    for el in w.findall('plugin') + w.findall('gui'):
        w.remove(el)
        if el.tag == 'plugin':
            avvisi.append(f"tolto il plugin del mondo {el.get('filename')} (li carica server.config di PX4)")
    fis = w.find('physics')
    if fis is None:
        fis = ET.SubElement(w, 'physics')
    fis.attrib = {'type': 'ode'}
    for c in list(fis):
        fis.remove(c)
    for tag, val in (('max_step_size', '0.004'), ('real_time_factor', '1.0'), ('real_time_update_rate', '250')):
        ET.SubElement(fis, tag).text = val
    if w.find('spherical_coordinates') is None:
        w.append(ET.fromstring(COORDINATE))
    if w.find('light') is None and not any(i.findtext('uri', '').strip().endswith('/sun') for i in w.findall('include')):
        w.append(ET.fromstring(LUCE))
    # URI relativi al file del mondo: la copia normalizzata vive altrove.
    for u in w.iter('uri'):
        t = (u.text or '').strip()
        if t and '://' not in t and not os.path.isabs(t):
            u.text = 'file://' + os.path.normpath(os.path.join(cartella_origine, t))
    return w


def punti_liberi(scena, xmin, ymin, xmax, ymax, passo=0.5, bordo=1.0):
    import open3d as o3d
    xs = np.arange(xmin + bordo, xmax - bordo + 1e-6, passo)
    ys = np.arange(ymin + bordo, ymax - bordo + 1e-6, passo)
    if not len(xs) or not len(ys):
        return np.zeros((0, 2)), np.zeros(0)
    X, Y = np.meshgrid(xs, ys)
    P = np.c_[X.ravel(), Y.ravel()]
    # Spazio libero attorno alla verticale fra 0.3 e 1.3 m: decollo (1 m) e giro
    # iniziale. Piu' in alto c'e' il soffitto degli uffici (2.27 m), che non e' un
    # problema per partire.
    gioco = np.full(len(P), np.inf)
    for z in (0.3, 0.8, 1.3):
        q = np.c_[P, np.full(len(P), z)].astype(np.float32)
        gioco = np.minimum(gioco, scena.compute_distance(o3d.core.Tensor(q)).numpy())
    # niente sopra la partenza sotto 1.9 m (un tavolo, uno scaffale, un balcone)
    raggi = np.c_[P, np.full(len(P), 0.3), np.zeros((len(P), 2)), np.ones(len(P))].astype(np.float32)
    su = scena.cast_rays(o3d.core.Tensor(raggi))['t_hit'].numpy()
    gioco[su < 1.6] = 0.0
    return P, gioco


def scegli_partenze(P, gioco, n, richieste):
    if richieste:
        out = []
        for s in richieste.split(';'):
            v = [float(x) for x in s.split(',')]
            out.append((v[0], v[1], v[2] if len(v) > 2 else 0.0))
        if len(out) < n:
            sys.exit(f'ERRORE: SPAWN ha {len(out)} pose, ne servono {n}')
        return out[:n]
    for soglia in (1.5, 1.2, 1.0, 0.8):
        ok = gioco >= soglia
        if ok.sum() >= n:
            break
    else:
        sys.exit('ERRORE: nessun punto di partenza libero (>= 0.8 m dagli ostacoli) nella zona da '
                 'esplorare: scegline uno con SPAWN="x,y"')
    C = P[ok]
    centro = C.mean(0)
    primo = C[np.argmin(np.linalg.norm(C - centro, axis=1))]
    scelte = [primo]
    libero = P[gioco >= min(soglia, 1.0)]
    for k in range(1, n):
        # in fila a 3 m dal primo, alternando i lati; se non e' libero, il punto
        # libero piu' vicino al primo e ad almeno 2.5 m da tutte le partenze
        cand = primo + np.array([0.0, 3.0 * ((k + 1) // 2) * (1 if k % 2 else -1)])
        d_lib = np.linalg.norm(libero - cand, axis=1).min() if len(libero) else np.inf
        if d_lib > 0.3 or min(np.linalg.norm(cand - s) for s in scelte) < 2.5:
            lontani = [p for p in libero if min(np.linalg.norm(p - s) for s in scelte) >= 2.5]
            if not lontani:
                sys.exit(f'ERRORE: non trovo {n} partenze libere a 2.5 m l\'una dall\'altra')
            lontani = np.array(lontani)
            cand = lontani[np.argmin(np.linalg.norm(lontani - primo, axis=1))]
        scelte.append(cand)
    return [(float(p[0]), float(p[1]), 0.0) for p in scelte]


def e_aperto(scena, P, gioco, lato):
    import open3d as o3d
    ok = P[gioco >= 1.0]
    if not len(ok):
        return False, 0.0
    sel = ok[np.linspace(0, len(ok) - 1, min(30, len(ok))).astype(int)]
    a = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    D = np.c_[np.cos(a), np.sin(a), np.zeros_like(a)]
    raggi = np.vstack([np.c_[np.tile([x, y, 1.2], (len(D), 1)), D] for x, y in sel]).astype(np.float32)
    hit = scena.cast_rays(o3d.core.Tensor(raggi))['t_hit'].numpy()
    fuga = float(np.mean(~np.isfinite(hit) | (hit > lato + 10)))
    return fuga > 0.05, fuga


def soffitto(scena, P, gioco):
    import open3d as o3d
    ok = P[gioco >= 1.0]
    if not len(ok):
        return False
    sel = ok[np.linspace(0, len(ok) - 1, min(30, len(ok))).astype(int)]
    raggi = np.c_[sel, np.full(len(sel), 0.3), np.zeros((len(sel), 2)), np.ones(len(sel))].astype(np.float32)
    su = scena.cast_rays(o3d.core.Tensor(raggi))['t_hit'].numpy()
    return bool(np.mean(su < 4.0) > 0.5)


def recinto(xmin, ymin, xmax, ymax, h=4.0, s=0.3):
    lx, ly = xmax - xmin, ymax - ymin
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    pezzi = [('nord', cx, ymax + s / 2, lx + 2 * s, s), ('sud', cx, ymin - s / 2, lx + 2 * s, s),
             ('est', xmax + s / 2, cy, s, ly), ('ovest', xmin - s / 2, cy, s, ly)]
    link = ''.join(f"""<link name="{n}"><pose>{x:.3f} {y:.3f} {h / 2} 0 0 0</pose>
  <collision name="c"><geometry><box><size>{a:.3f} {b:.3f} {h}</size></box></geometry></collision>
  <visual name="v"><geometry><box><size>{a:.3f} {b:.3f} {h}</size></box></geometry>
    <material><ambient>0.55 0.55 0.5 1</ambient><diffuse>0.55 0.55 0.5 1</diffuse></material></visual>
</link>""" for n, x, y, a, b in pezzi)
    return ET.fromstring(f'<model name="c2x_recinto"><static>true</static>{link}</model>')


def config_generica(xmin, ymin, xmax, ymax, zmax, nome, origine):
    testo = open(os.path.join(CONFIG, CONFIG_BASE)).read()
    m = 1.5
    valori = {'sdf_map.map_size_x': xmax - xmin + 2 * m, 'sdf_map.map_size_y': ymax - ymin + 2 * m,
              'sdf_map.box_min_x': xmin, 'sdf_map.box_min_y': ymin,
              'sdf_map.box_max_x': xmax, 'sdf_map.box_max_y': ymax, 'sdf_map.box_max_z': zmax,
              # celle di spartizione fra droni: ~4 per lato, come su cubicle_office (7.5 m su 31)
              'partitioning.grid_size': float(np.clip(round(max(xmax - xmin, ymax - ymin) / 4 * 2) / 2, 4.0, 10.0))}
    for k, v in valori.items():
        testo, n = re.subn(rf'^(\s*){re.escape(k)}:.*$', rf'\g<1>{k}: {v:.2f}', testo, count=1, flags=re.M)
        if n != 1:
            sys.exit(f'ERRORE: {k} non trovato in {CONFIG_BASE}')
    testo = re.sub(r'^(\s*)sdf_map\.map_size_x:(.*)$',
                   rf'\g<1>sdf_map.map_origin_x: {xmin - m:.2f}\n\g<1>sdf_map.map_origin_y: {ymin - m:.2f}\n'
                   r'\g<1>sdf_map.map_size_x:\2', testo, count=1, flags=re.M)
    testa = (f'# GENERATO DA scripts/prepara_mondo.py per il mondo "{nome}" ({origine}).\n'
             f'# Base: {CONFIG_BASE}; cambiano solo zona da esplorare, mappa e celle di spartizione.\n')
    return testa + testo


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mondo')
    ap.add_argument('--droni', type=int, default=1)
    ap.add_argument('--uscita', required=True)
    a = ap.parse_args()
    os.makedirs(a.uscita, exist_ok=True)
    origine = trova_mondo(a.mondo)
    nome = nome_pulito(origine)
    avvisi = []
    albero = leggi(origine)
    w = normalizza(albero, nome, os.path.dirname(origine), avvisi)
    radici = gm.radici_risorse(origine)
    info = {'nome': nome, 'origine': origine, 'droni': a.droni}

    if nome in TARATI and not os.environ.get('SPAWN') and not os.environ.get('BOX'):
        pose, cfg = TARATI[nome]
        if a.droni > len(pose):
            sys.exit(f'ERRORE: per {nome} sono tarate {len(pose)} partenze')
        partenze = [tuple(float(v) for v in p.split(',')) + (0.0,) for p in pose[:a.droni]]
        testo_cfg = open(os.path.join(CONFIG, cfg)).read()
        info.update(tarato=True, config_origine=cfg)
    else:
        geo = gm.carica(origine, radici)
        avvisi += geo.avvisi
        terra = geo.terra or [0.0]
        if min(abs(z) for z in terra) > 0.05:
            sys.exit(f'ERRORE: il terreno del mondo e\' a quota {terra}, lo stack vola a quote assolute '
                     'con il terreno a 0')
        if not geo.terra:
            # Nessun piano di terra: se ne aggiunge uno, o il drone cadrebbe nel vuoto.
            w.append(ET.fromstring('<model name="c2x_terreno"><static>true</static><link name="l">'
                                   '<collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size>'
                                   '</plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1'
                                   '</normal><size>200 200</size></plane></geometry><material><ambient>0.5 0.5 0.5 1'
                                   '</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model>'))
            avvisi.append('nessun piano di terra: aggiunto a quota 0')
        V, F = geo.tutti()
        scena = geo.scena()
        if os.environ.get('BOX'):
            xmin, ymin, xmax, ymax = [float(v) for v in os.environ['BOX'].split(',')]
        else:
            if not len(F):
                sys.exit('ERRORE: il mondo non ha ostacoli: indica la zona da esplorare con BOX="xmin,ymin,xmax,ymax"')
            T = V[F]
            fascia = (T[:, :, 2].min(1) < 2.5) & (T[:, :, 2].max(1) > 0.1)
            if not fascia.any():
                sys.exit('ERRORE: nessun ostacolo fra 0.1 e 2.5 m di quota: indica la zona da esplorare '
                         'con BOX="xmin,ymin,xmax,ymax"')
            Q = T[fascia].reshape(-1, 3)
            xmin, ymin = Q[:, 0].min() - 0.5, Q[:, 1].min() - 0.5
            xmax, ymax = Q[:, 0].max() + 0.5, Q[:, 1].max() + 0.5
            for lo, hi, ax in ((xmin, xmax, 0), (ymin, ymax, 1)):
                if hi - lo > LATO_MAX:
                    c = (lo + hi) / 2
                    avvisi.append(f'zona da esplorare ridotta a {LATO_MAX:.0f} m lungo {"xy"[ax]} '
                                  '(BOX per sceglierla)')
                    if ax == 0:
                        xmin, xmax = c - LATO_MAX / 2, c + LATO_MAX / 2
                    else:
                        ymin, ymax = c - LATO_MAX / 2, c + LATO_MAX / 2
        P, gioco = punti_liberi(scena, xmin, ymin, xmax, ymax)
        lato = max(xmax - xmin, ymax - ymin)
        aperto, fuga = e_aperto(scena, P, gioco, lato)
        forza = os.environ.get('RECINTO', 'auto')
        metti_recinto = aperto if forza == 'auto' else forza == '1'
        if metti_recinto:
            # Il recinto sta 1.5 m fuori dall'ingombro: spazio per girare attorno agli
            # ostacoli di bordo. La zona da esplorare e' il suo interno.
            xmin, ymin, xmax, ymax = xmin - 1.0, ymin - 1.0, xmax + 1.0, ymax + 1.0
            r = recinto(xmin - 0.2, ymin - 0.2, xmax + 0.2, ymax + 0.2)
            w.append(r)
            gm._modello(r, np.eye(4), 'c2x_recinto', a.uscita, radici, geo)
            scena = geo.scena()
            P, gioco = punti_liberi(scena, xmin, ymin, xmax, ymax)
        coperto = soffitto(scena, P, gioco)
        zmax = 2.5 if coperto else 2.2
        partenze = scegli_partenze(P, gioco, a.droni, os.environ.get('SPAWN', ''))
        for x, y, _ in partenze:
            d = float(geo.distanza(np.array([[x, y, z] for z in (0.3, 0.8, 1.3)])).min())
            if d < 0.8:
                avvisi.append(f'partenza ({x:.1f},{y:.1f}) a soli {d:.2f} m da un ostacolo')
        testo_cfg = config_generica(xmin, ymin, xmax, ymax, zmax, nome, origine)
        info.update(tarato=False, zona=[round(v, 2) for v in (xmin, ymin, xmax, ymax)], quota_zona=zmax,
                    aperto=bool(aperto), raggi_in_fuga=round(fuga, 3), recinto=bool(metti_recinto),
                    soffitto=coperto, config_origine=CONFIG_BASE)

    sdf_out = os.path.join(a.uscita, nome + '.sdf')
    albero.write(sdf_out, encoding='unicode', xml_declaration=True)
    cfg_out = os.path.join(a.uscita, 'config.yaml')
    open(cfg_out, 'w').write(testo_cfg)
    pose = [f'{x:.3f},{y:.3f},{QUOTA_APPOGGIO},0,0,{yaw:.4f}' for x, y, yaw in partenze]
    info.update(partenze=pose, avvisi=avvisi, sdf=sdf_out, config=cfg_out)
    json.dump(info, open(os.path.join(a.uscita, 'mondo.json'), 'w'), indent=1)
    risorse = [a.uscita] + radici
    with open(os.path.join(a.uscita, 'mondo.env'), 'w') as f:
        f.write(f'MONDO_NOME={shlex.quote(nome)}\n')
        f.write(f'MONDO_ORIGINE={shlex.quote(origine)}\n')
        f.write(f'MONDO_SDF={shlex.quote(sdf_out)}\n')
        f.write(f'MONDO_CONFIG={shlex.quote(cfg_out)}\n')
        f.write(f'MONDO_RISORSE={shlex.quote(":".join(risorse))}\n')
        f.write('SPAWN_POSES=(' + ' '.join(shlex.quote(p) for p in pose) + ')\n')
    print(f'mondo {nome} ({origine})')
    if info.get('tarato'):
        print(f'  tarato: partenze e configurazione da {info["config_origine"]}')
    else:
        print(f'  zona da esplorare x {xmin:.1f}..{xmax:.1f}  y {ymin:.1f}..{ymax:.1f}  quota fino a {zmax} m'
              f' ({"soffitto" if info["soffitto"] else "cielo aperto"})')
        print(f'  {"APERTO" if info["aperto"] else "chiuso"} ({100 * info["raggi_in_fuga"]:.0f}% dei raggi '
              f'orizzontali senza ostacoli){": aggiunto un recinto alto 4 m" if info["recinto"] else ""}')
    print('  partenze: ' + '  '.join(p.rsplit(',', 3)[0] for p in pose))
    for av in avvisi:
        print('  AVVISO:', av)
    return 0


if __name__ == '__main__':
    sys.exit(main())
