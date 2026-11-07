#!/usr/bin/env python3
"""Genera il mondo Gazebo "parco": un parco di quartiere all'aperto di 31 x 31 m
(961 m2) composto con modelli ufficiali del catalogo Gazebo Fuel (copiati in
src/c2x_worlds/models_parco, licenze in LICENZE.txt).

Perche' composto e non scaricato gia' pronto (17 ottobre 2026): nessun mondo
pubblico trovato era insieme all'aperto, realistico, entro 1000 m2 e volabile fra
1.0 e 1.7 m. Il frutteto di Clearpath (olivi bassi, chiome a 0.4-2.0 m) lascia
fra i filari 0.34-0.50 m liberi dal centro del corridoio, contro i 0.65 che servono
al drone; il bosco di PX4 e' 1726 m2 di soli alberi; Harmonic World, Baylands e
Nathan Benderson Park sono troppo grandi o pesanti per il Jetson.

Pianta (ENU, x verso est, y verso nord, origine al centro):
  sud     strada asfaltata (y -15.5..-9.5) con SUV e due utilitarie parcheggiate,
          marciapiede (y -9.5..-7.5) con lampioni, idrante, cassetta postale, stop;
          oltre la strada una fila di case (fuori dall'area esplorata);
  nord-o  area giochi;   centro  fontana, panchine, cestini;   sparsi  querce e pini;
  confine siepe alta 4 m su ovest, nord ed est, con una casa oltre la siepe nord. Serve come il recinto di
          open_field: in campo aperto i raggi orizzontali del lidar non tornano,
          lo spazio non viene mai marcato libero e l'esplorazione non si chiude.
          4 m, ben sopra la box di esplorazione (2.2 m): anche le celle alte vicino
          al drone vengono attraversate da raggi che finiscono sulla siepe.

Scrive:
  src/c2x_worlds/worlds/parco.sdf          il mondo
  src/c2x_worlds/worlds/parco_ostacoli.npz pianta VERA degli ostacoli nella fascia
                                           di volo (0.4-1.9 m), dalle geometrie di
                                           collisione dei modelli, e gli stessi
                                           ostacoli in 3D (punti3d, scatole3d): la
                                           usano le analisi del margine
                                           (analisi_campagna.py)
  src/c2x_worlds/worlds/parco_pianta.png   controllo visivo (verde = raggiungibile)
  src/c2x_worlds/models_parco/*/meshes/*_lidar.dae e il visual per_lidar nei
                                           model.sdf: la collisione resa visibile al
                                           lidar (vedi geometrie_per_lidar)

Uso: python3 make_parco.py
"""
import os
import sys

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
from modelli_geo import punti_collisione, triangoli_dae  # noqa: E402

WS = os.path.dirname(QUI)
MONDI = os.path.join(WS, 'src', 'c2x_worlds', 'worlds')
MODELLI = os.path.join(WS, 'src', 'c2x_worlds', 'models_parco')

MEZZO = 15.5          # semilato dell'area esplorata (31 x 31 m)
SIEPE_H = 4.0         # altezza della siepe di confine
SIEPE_S = 0.8         # spessore
STRADA = (-15.5, -9.5)      # y della strada
MARCIAPIEDE = (-9.5, -7.5)  # y del marciapiede
BANDA = (0.4, 1.9)    # fascia di quota della pianta degli ostacoli
PARTENZA = (-13.0, -5.5)    # posa di decollo del drone (deve restare libera)

# (nome istanza, cartella del modello, x, y, imbardata)
OGGETTI = [
    # oltre la strada: case (la facciata fa da confine sud)
    ('casa_sud_ovest', 'house_3', -7.5, -19.95, 0.0),
    ('casa_sud_est', 'house_1', 7.5, -21.7, 0.0),
    # strada: auto parcheggiate
    ('suv', 'suv', -9.0, -11.3, 0.0),
    ('utilitaria', 'hatchback', -1.0, -11.3, 0.0),
    ('utilitaria_rossa', 'hatchback_red', 8.0, -13.8, np.pi),
    # marciapiede
    ('lampione_1', 'lamp_post', -12.0, -8.0, 0.0),
    ('lampione_2', 'lamp_post', 0.0, -8.0, 0.0),
    ('lampione_3', 'lamp_post', 12.0, -8.0, 0.0),
    ('idrante', 'fire_hydrant', -5.0, -8.2, 0.0),
    ('cassetta_postale', 'mailbox', 5.0, -8.0, 0.0),
    ('stop', 'stop_sign', 14.5, -8.3, np.pi / 2),
    # parco
    ('area_giochi', 'playground', -9.0, 10.5, 0.0),
    # oltre la siepe nord: una casa (dentro l'area, con l'interno chiuso visto dalle
    # finestre, lasciava frontiere irraggiungibili)
    ('casa_nord', 'house_2', 6.6, 19.8, 0.0),
    ('quercia_nord_est', 'Oak tree', 8.5, 8.5, 1.0),
    ('pino_4', 'Pine Tree', 12.5, 12.5, 0.0),
    ('panchina_nord', 'FoodCourtBenchLong', 3.5, 11.0, np.pi / 2),
    ('fontana', 'fountain', -1.0, 0.0, 0.0),
    ('panchina_ovest', 'FoodCourtBenchLong', -4.0, 0.0, 0.0),
    ('panchina_est', 'FoodCourtBenchLong', 2.0, 0.0, np.pi),
    ('cestino_1', 'TrashBin', -4.0, -1.6, 0.0),
    ('cestino_2', 'TrashBin', 2.0, 1.6, 0.0),
    ('quercia_est', 'Oak tree', 9.5, -3.5, 0.0),
    ('quercia_ovest', 'Oak tree', -12.0, -1.0, 2.0),
    ('pino_1', 'Pine Tree', -5.5, 5.0, 0.0),
    ('pino_2', 'Pine Tree', 4.5, -4.0, 0.0),
    ('pino_3', 'Pine Tree', -1.5, 10.0, 0.0),
]

TESTA = """<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATO DA scripts/make_parco.py - non modificare a mano. -->
<sdf version="1.9">
  <world name="parco">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    <!-- Nessun blocco <plugin>: li carica il server.config di PX4 (vedi open_field). -->
    <light type="directional" name="sole">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.95 0.95 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.3 0.2 -0.9</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""


def scatola(nome, cx, cy, cz, sx, sy, sz, colore, collisione=True, texture=None):
    mat = (f"<pbr><metal><albedo_map>{texture}</albedo_map></metal></pbr>" if texture else '')
    col = (f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
           if collisione else '')
    return f"""    <model name="{nome}">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 0 0</pose>
      <link name="link">
        {col}
        <visual name="v">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{colore}</ambient><diffuse>{colore}</diffuse>{mat}</material>
        </visual>
      </link>
    </model>
"""


def siepi():
    """Siepe di confine su ovest, nord, est (il sud e' chiuso dalle case)."""
    m, s, h = MEZZO, SIEPE_S, SIEPE_H
    verde = '0.18 0.32 0.12 1'
    L = 2 * m + 2 * s
    return [
        ('siepe_ovest', -m - s / 2, 0.0, h / 2, s, L, h, verde),
        ('siepe_est', m + s / 2, 0.0, h / 2, s, L, h, verde),
        ('siepe_nord', 0.0, m + s / 2, h / 2, L, s, h, verde),
        # spezzoni fra la strada e le case, alle estremita' del lato sud
        ('siepe_sud_ovest', -m - s / 2, -m - 3.0, h / 2, s, 6.0, h, verde),
        ('siepe_sud_est', m + s / 2, -m - 3.0, h / 2, s, 6.0, h, verde),
    ]


def oggetti():
    # PARCO_SENZA_ALBERI=1: stesso parco senza querce e pini (prova del 17 ottobre
    # 2026 sulla deriva di FAST-LIO vicino alle chiome fatte di rettangoli trasparenti).
    if os.environ.get('PARCO_SENZA_ALBERI') == '1':
        return [o for o in OGGETTI if o[1] not in ('Oak tree', 'Pine Tree')]
    return OGGETTI


def scrivi_mondo():
    parti = [TESTA]
    # terreno erboso grande (oltre l'area, per la vista della telecamera)
    parti.append("""    <model name="prato">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <material><ambient>0.33 0.45 0.22 1</ambient><diffuse>0.33 0.45 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>
""")
    lun = 2 * MEZZO + 2 * SIEPE_S + 12.0
    y0, y1 = STRADA
    parti.append(scatola('strada', 0.0, (y0 + y1) / 2 - 1.5, 0.01, lun, (y1 - y0) + 3.0, 0.02,
                         '0.35 0.35 0.35 1', collisione=False,
                         texture='model://asphalt_plane/materials/textures/tarmac.png'))
    m0, m1 = MARCIAPIEDE
    parti.append(scatola('marciapiede', 0.0, (m0 + m1) / 2, 0.06, lun, m1 - m0, 0.12, '0.62 0.62 0.6 1'))
    # vialetti di ghiaia (solo aspetto, sotto la fascia di volo)
    parti.append(scatola('vialetto_nord_sud', -1.0, -3.75, 0.005, 1.6, 7.5, 0.01, '0.72 0.66 0.52 1', collisione=False))
    parti.append(scatola('vialetto_est_ovest', -1.0, 0.0, 0.005, 26.0, 1.6, 0.01, '0.72 0.66 0.52 1', collisione=False))
    for s in siepi():
        parti.append(scatola(*s))
    for nome, cartella, x, y, yaw in oggetti():
        parti.append(f"""    <include>
      <uri>model://{cartella}</uri>
      <name>{nome}</name>
      <pose>{x} {y} 0 0 0 {yaw:.6f}</pose>
      <static>true</static>
    </include>
""")
    parti.append("  </world>\n</sdf>\n")
    out = os.path.join(MONDI, 'parco.sdf')
    open(out, 'w').write(''.join(parti))
    return out


def pianta_ostacoli(res=0.05, lato=40.0):
    """Occupazione 2D nella fascia BANDA dalle collisioni vere, centrata nell'origine."""
    n = int(lato / res)
    occ = np.zeros((n, n), bool)
    x0 = y0 = -lato / 2

    def marca(P):
        P = P[(P[:, 2] >= BANDA[0]) & (P[:, 2] <= BANDA[1])]
        i = ((P[:, 0] - x0) / res).astype(int)
        j = ((P[:, 1] - y0) / res).astype(int)
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        occ[i[ok], j[ok]] = True

    for nome, cartella, x, y, yaw in oggetti():
        P = punti_collisione(os.path.join(MODELLI, cartella), [MODELLI])
        c, s = np.cos(yaw), np.sin(yaw)
        W = np.c_[c * P[:, 0] - s * P[:, 1] + x, s * P[:, 0] + c * P[:, 1] + y, P[:, 2]]
        marca(W)
    for nome, cx, cy, cz, sx, sy, sz, _ in siepi():
        i0 = int((cx - sx / 2 - x0) / res); i1 = int((cx + sx / 2 - x0) / res)
        j0 = int((cy - sy / 2 - y0) / res); j1 = int((cy + sy / 2 - y0) / res)
        occ[max(0, i0):min(n, i1 + 1), max(0, j0):min(n, j1 + 1)] = True
    return occ, x0, y0, res


INIZIO_LIDAR = '      <!-- INIZIO per_lidar: GENERATO DA scripts/make_parco.py (geometrie_per_lidar) -->\n'
FINE_LIDAR = '      <!-- FINE per_lidar -->\n'


def _scrivi_dae_doppio(out, V, T, origine):
    """COLLADA con i triangoli T nei due versi e le normali per faccia. Senza normali
    Gazebo disegnava la mesh bianca e opaca in telecamera, trasparenza ignorata (lo
    stesso con un OBJ)."""
    T2 = np.vstack([T, T[:, ::-1]])
    N = np.cross(V[T2[:, 1]] - V[T2[:, 0]], V[T2[:, 2]] - V[T2[:, 0]])
    N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
    pos = ' '.join(f'{v:.6f}' for v in V.ravel())
    nrm = ' '.join(f'{v:.5f}' for v in N.ravel())
    idx = ' '.join(f'{a} {k} {b} {k} {c} {k}' for k, (a, b, c) in enumerate(T2))
    with open(out, 'w') as f:
        f.write(f"""<?xml version="1.0" encoding="utf-8"?>
<!-- GENERATO DA scripts/make_parco.py (geometrie_per_lidar) da {origine}: stessa
     geometria della collisione, ogni triangolo nei due versi. -->
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_effects><effect id="lidar-effect"><profile_COMMON><technique sid="common"><lambert>
    <diffuse><color sid="diffuse">0.5 0.5 0.5 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="lidar-material" name="lidar"><instance_effect url="#lidar-effect"/></material></library_materials>
  <library_geometries><geometry id="lidar-mesh" name="lidar"><mesh>
    <source id="lidar-pos"><float_array id="lidar-pos-array" count="{V.size}">{pos}</float_array>
      <technique_common><accessor source="#lidar-pos-array" count="{len(V)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="lidar-nrm"><float_array id="lidar-nrm-array" count="{N.size}">{nrm}</float_array>
      <technique_common><accessor source="#lidar-nrm-array" count="{len(N)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <vertices id="lidar-vtx"><input semantic="POSITION" source="#lidar-pos"/></vertices>
    <triangles material="lidar-material" count="{len(T2)}"><input semantic="VERTEX" source="#lidar-vtx" offset="0"/>
      <input semantic="NORMAL" source="#lidar-nrm" offset="1"/>
      <p>{idx}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scena" name="scena"><node id="lidar" name="lidar" type="NODE">
    <instance_geometry url="#lidar-mesh"><bind_material><technique_common>
      <instance_material symbol="lidar-material" target="#lidar-material"/></technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scena"/></scene>
</COLLADA>
""")


def geometrie_per_lidar():
    """Fa vedere al lidar ESATTAMENTE cio' che collide, in ogni modello del parco
    (18 ottobre 2026).

    Il lidar simulato di Gazebo non vede tutto cio' con cui il drone puo' urtare:
    rispetta la trasparenza delle texture (le chiome di pini e querce sono rettangoli
    a texture trasparente) e scarta le facce posteriori dei triangoli anche con
    double_sided, mentre la collisione e' la mesh intera, nei due versi. Misurato con
    lidar fermi identici a quello del drone, raggio per raggio contro la geometria di
    collisione: chiome viste al 15% (pino) e al 20% (quercia); la casa a sud-ovest,
    che verso il parco mostra solo facce posteriori, all'1%; il retro dello stop al
    26%. Il drone ha urtato un ramo di pino assente dalla mappa (run
    2026-10-18_0155) ed e' passato a 0.31 m dalla facciata di quella casa (run
    2026-10-18_1055).

    Qui per ogni collisione a mesh si scrive meshes/<mesh>_lidar.dae (la stessa
    geometria, ogni triangolo nei due versi) e la si aggiunge al model.sdf come visual
    con trasparenza 1, senza ombre, stessa posa e scala della collisione: le telecamere
    e la GUI non la disegnano (immagini identiche pixel per pixel; con le ombre accese
    la chioma trasparente proiettava comunque un'ombra piena), il lidar si', perche'
    ignora la trasparenza del materiale. Il blocco nel model.sdf sta fra i commenti
    INIZIO/FINE per_lidar e si riscrive a ogni esecuzione."""
    import xml.etree.ElementTree as ET
    from modelli_geo import risolvi_uri, triangoli_obj
    for cartella in sorted({o[1] for o in OGGETTI}):
        d = os.path.join(MODELLI, cartella)
        f_sdf = os.path.join(d, 'model.sdf')
        testo = open(f_sdf).read()
        if INIZIO_LIDAR in testo:
            testo = testo[:testo.index(INIZIO_LIDAR)] + testo[testo.index(FINE_LIDAR) + len(FINE_LIDAR):]
        collisioni = [c for link in ET.fromstring(testo).find('model').findall('link')
                      for c in link.findall('collision')]
        assert len(collisioni) == 1 and testo.count('</collision>') == 1, cartella
        col = collisioni[0]
        mesh = col.find('geometry/mesh')
        assert mesh is not None, (cartella, 'collisione non a mesh')
        f = risolvi_uri(mesh.findtext('uri'), d, [MODELLI])
        V, T = triangoli_dae(f) if f.lower().endswith('.dae') else triangoli_obj(f)
        nome = os.path.splitext(os.path.basename(f))[0] + '_lidar.dae'
        os.makedirs(os.path.join(d, 'meshes'), exist_ok=True)
        _scrivi_dae_doppio(os.path.join(d, 'meshes', nome), V, T, os.path.basename(f))
        posa = f'        <pose>{col.findtext("pose").strip()}</pose>\n' if col.findtext('pose') else ''
        scala = f'<scale>{mesh.findtext("scale").strip()}</scale>' if mesh.findtext('scale') else ''
        blocco = (INIZIO_LIDAR +
                  "      <!-- La collisione come la deve vedere il lidar: vedi la funzione in make_parco.py. -->\n"
                  '      <visual name="per_lidar">\n' + posa +
                  '        <cast_shadows>false</cast_shadows>\n' +
                  f'        <geometry><mesh><uri>model://{cartella}/meshes/{nome}</uri>{scala}</mesh></geometry>\n'
                  '        <material><diffuse>0.5 0.5 0.5 1</diffuse></material>\n'
                  '        <transparency>1.0</transparency>\n'
                  '      </visual>\n' + FINE_LIDAR)
        i = testo.index('</collision>') + len('</collision>\n')
        open(f_sdf, 'w').write(testo[:i] + blocco + testo[i:])
        print(f'{cartella}: {len(T)} triangoli x 2 -> meshes/{nome}')


def ostacoli_3d(passo=0.05):
    """Ostacoli VERI in 3D per il margine di analisi_campagna: punti di collisione
    dei modelli (un punto per cubetto di 'passo') e le siepi come scatole
    (cx, cy, cz, hx, hy, hz). La pianta 2D da sola non basta all'aperto: proietta
    nella fascia 0.4-1.9 m il tetto delle auto (1.0 m) e la chioma delle querce
    (da 1.5 m), sopra e sotto i quali il drone passa davvero. Run del 18 ottobre
    2026: margine 2D 0.00 m (5.7% del volo "in contatto"), margine 3D 0.50 m."""
    P = []
    for nome, cartella, x, y, yaw in oggetti():
        Q = punti_collisione(os.path.join(MODELLI, cartella), [MODELLI])
        c, s = np.cos(yaw), np.sin(yaw)
        P.append(np.c_[c * Q[:, 0] - s * Q[:, 1] + x, s * Q[:, 0] + c * Q[:, 1] + y, Q[:, 2]])
    P = np.vstack(P)
    P = P[P[:, 2] > 0.05]   # il terreno non e' un ostacolo
    _, k = np.unique(np.floor(P / passo).astype(np.int64), axis=0, return_index=True)
    scatole = np.array([(cx, cy, cz, sx / 2, sy / 2, sz / 2)
                        for _, cx, cy, cz, sx, sy, sz, _ in siepi()])
    return P[np.sort(k)].astype(np.float32), scatole


def main():
    out = scrivi_mondo()
    print('mondo:', out)
    geometrie_per_lidar()
    occ, x0, y0, res = pianta_ostacoli()
    sys.path.insert(0, QUI)
    import clearance_report as cr
    dist = cr.edt(occ) * res
    punti3d, scatole3d = ostacoli_3d()
    np.savez_compressed(os.path.join(MONDI, 'parco_ostacoli.npz'), occ=occ, dist=dist,
                        x0=x0, y0=y0, res=res, banda=np.array(BANDA),
                        punti3d=punti3d, scatole3d=scatole3d)
    print('pianta degli ostacoli:', os.path.join(MONDI, 'parco_ostacoli.npz'))
    # Controlli: decollo libero e spazio raggiungibile col drone gonfiato (0.65 m)
    def d_in(x, y):
        return dist[int((x - x0) / res), int((y - y0) / res)]
    print(f'decollo {PARTENZA}: {d_in(*PARTENZA):.2f} m dall\'ostacolo piu\' vicino')
    from scipy import ndimage
    libero = dist >= 0.65
    area = np.zeros_like(libero)
    i0 = int((-MEZZO - x0) / res); i1 = int((MEZZO - x0) / res)
    area[i0:i1, i0:i1] = True
    lab, _ = ndimage.label(libero & area)
    k = lab[int((PARTENZA[0] - x0) / res), int((PARTENZA[1] - y0) / res)]
    raggiungibile = lab == k
    print(f'area libera (>=0.65 m dagli ostacoli) raggiungibile dal decollo: '
          f'{raggiungibile.sum() * res * res:.0f} m2 su {(libero & area).sum() * res * res:.0f} m2 liberi')
    try:
        from PIL import Image
        img = np.zeros(occ.shape + (3,), np.uint8)
        img[...] = (np.clip(dist / 3.0, 0, 1) * 200 + 40).astype(np.uint8)[..., None]
        img[occ] = (0, 0, 0)
        img[raggiungibile] = (img[raggiungibile] * np.array([0.6, 1.0, 0.6])).astype(np.uint8)
        img[(dist < 0.65) & ~occ & area] = (200, 120, 60)
        Image.fromarray(np.transpose(img, (1, 0, 2))[::-1]).resize((800, 800)).save(os.path.join(MONDI, 'parco_pianta.png'))
        print('controllo visivo: ' + os.path.join(MONDI, 'parco_pianta.png') + '')
    except Exception as e:  # pragma: no cover
        print('immagine non generata:', e)


if __name__ == '__main__':
    main()
