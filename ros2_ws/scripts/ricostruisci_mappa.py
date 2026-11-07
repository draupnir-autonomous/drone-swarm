#!/usr/bin/env python3
"""MAPPA REALISTICA: superficie 3D continua e colorata dell'ambiente esplorato.

Parte dalla cartella prodotta da registra_mappa.py durante il volo:
  1. geometria: i punti lidar accumulati a 2 cm (gia' posati con la posa del loro
     istante) diventano una superficie continua con la ricostruzione di Poisson.
     Le normali sono orientate verso il lato da cui il lidar ha visto ogni punto,
     cioe' verso lo spazio libero. Le parti che Poisson inventa per chiudere i
     buchi (zone mai viste) si tagliano via: si tengono solo i triangoli a pochi
     centimetri da un punto misurato;
  2. colore: ogni vertice viene proiettato nei fotogrammi chiave della
     telecamera di bordo e prende il colore medio dei fotogrammi da cui e'
     DAVVERO visibile (controllo di occlusione con un raggio dalla telecamera al
     vertice contro la superficie stessa). I vertici che la telecamera non ha mai
     visto restano grigi;
  3. uscite: mappa_realistica.ply (superficie colorata), nuvola_colorata.ply
     (punti colorati), anteprime PNG (dall'alto e in prospettiva).

Uso: ricostruisci_mappa.py [cartella] [--depth 9] [--max_triangoli 1500000]
"""
import argparse
import json
import os
import platform
import sys

import numpy as np
import open3d as o3d


def carica_punti(cartella, min_conteggio):
    d = np.load(os.path.join(cartella, 'punti.npz'))
    m = d['conteggi'] >= min_conteggio
    return d['centri'][m].astype(np.float64), d['direzioni'][m].astype(np.float64)


def superficie(punti, direzioni, depth, taglio, max_triangoli):
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(punti))
    # Raggio 15 cm: con 8 cm le superfici viste da lontano (pochi punti) non
    # avevano abbastanza vicini per una normale affidabile.
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    n = np.asarray(pc.normals)
    # Normale rivolta verso il lato da cui il punto e' stato osservato.
    capovolgi = np.einsum('ij,ij->i', n, direzioni) < 0
    n[capovolgi] *= -1
    pc.normals = o3d.utility.Vector3dVector(n)
    # Jetson (aarch64, Open3D 0.20): Poisson multi-thread abortiva ogni volta con
    # "Failed to close loop" (FEMTree.IsoSurface) e la mappa 3D non usciva mai. Con
    # un solo thread no. Sul PC x86 resta multi-thread come prima.
    mesh, _densita = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pc, depth=depth, linear_fit=True,
        n_threads=1 if platform.machine() in ('aarch64', 'arm64') else -1)
    # NIENTE taglio per densita': toglieva il 2% dei vertici meno sostenuti, cioe'
    # proprio le pareti viste da lontano o di sbieco (copertura delle superfici
    # osservabili 93.6% -> 98.1% senza, precisione invariata al 100% entro 15 cm).
    # Le parti inventate le toglie gia' il taglio per distanza qui sotto.
    # Via cio' che Poisson ha inventato: vertici lontani da ogni punto misurato.
    vertici = o3d.geometry.PointCloud(mesh.vertices)
    distanze = np.asarray(vertici.compute_point_cloud_distance(pc))
    mesh.remove_vertices_by_mask(distanze > taglio)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    # LEVIGATURA: il rumore del lidar (1 cm) ricostruito a pochi centimetri
    # rende le pareti "bitorzolute". Taubin appiattisce le superfici senza
    # restringere la forma, a differenza della media semplice.
    mesh = mesh.filter_smooth_taubin(number_of_iterations=10)
    # SEMPLIFICAZIONE: a 2 cm la prima prova aveva 9.2 milioni di triangoli e un
    # file da 354 MB, che il motore di anteprima non riusciva nemmeno a disegnare
    # (immagine nera). Le superfici di un interno sono quasi tutte piane, quindi
    # la decimazione toglie triangoli senza perdere forma.
    if len(mesh.triangles) > max_triangoli:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=max_triangoli)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh, pc


# ELICHE NELL'INQUADRATURA (code review, ottobre 2026, verificato sui fotogrammi): la
# telecamera frontale (0.10, 0, 0.12 dal centro, 8.6 gradi in giu', 90 gradi di campo)
# vede le pale delle due eliche anteriori (mozzi a +-0.174 m, pale da 10" a ~5 cm di
# quota) come tratti scuri negli angoli in basso dell'immagine, che finivano come strisce
# scure sul pavimento della mappa colorata. La telecamera e' fissa sul corpo, quindi la
# zona dove le pale possono passare e' sempre la stessa: la proiezione dei due dischi
# delle eliche (raggio 0.13 m, a 0.044 e 0.06 m), ~20% dell'immagine. Quei pixel non
# colorano: il pavimento li' attorno lo vedono comunque altri fotogrammi.
ELICHE_CAM_T = np.array([0.10, 0.0, 0.12])
ELICHE_CAM_PITCH = 0.15
ELICHE_MOZZI = ((0.174, 0.174), (0.174, -0.174))   # solo le anteriori: le altre stanno dietro
ELICHE_RAGGIO = 0.13
ELICHE_QUOTE = (0.044, 0.06)


def maschera_eliche(w, h, fx, fy, cx, cy):
    """Pixel (h x w, True) in cui possono comparire le pale delle eliche anteriori."""
    from PIL import Image, ImageDraw
    a = ELICHE_CAM_PITCH
    r = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    img = Image.new('L', (int(w), int(h)), 0)
    disegno = ImageDraw.Draw(img)
    for z in ELICHE_QUOTE:
        for mx, my in ELICHE_MOZZI:
            punti = []
            for th in np.linspace(0, 2 * np.pi, 360):
                p = np.array([mx + ELICHE_RAGGIO * np.cos(th), my + ELICHE_RAGGIO * np.sin(th), z])
                pc = r.T @ (p - ELICHE_CAM_T)
                if pc[0] < 0.02:
                    continue
                punti.append((cx - fx * pc[1] / pc[0], cy - fy * pc[2] / pc[0]))
            if len(punti) > 2:
                disegno.polygon(punti, fill=255)
    return np.asarray(img) > 0


def colora(mesh, cartella, max_dist, max_fotogrammi, eliche=True):
    # Senza fotogrammi (registra_mappa.py di serie non salva piu' la telecamera)
    # la superficie resta grigia.
    pose = os.path.join(cartella, 'fotogrammi', 'pose.json')
    info = json.load(open(pose))['fotogrammi'] if os.path.exists(pose) else []
    v = np.asarray(mesh.vertices)
    somma = np.zeros((len(v), 3))
    peso = np.zeros(len(v))
    if not info:
        return np.full((len(v), 3), 0.6), 0
    if len(info) > max_fotogrammi:
        info = [info[i] for i in np.linspace(0, len(info) - 1, max_fotogrammi).astype(int)]
    scena = o3d.t.geometry.RaycastingScene()
    scena.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    from PIL import Image
    maschere = {}
    for f in info:
        img = np.asarray(Image.open(os.path.join(cartella, 'fotogrammi', f['file'])).convert('RGB'))
        t, R = np.array(f['t']), np.array(f['R'])
        pc = (v - t) @ R              # coordinate nella terna della telecamera (x avanti)
        x = pc[:, 0]
        davanti = x > 0.1
        u = f['cx'] - f['fx'] * pc[:, 1] / np.where(davanti, x, 1)
        w = f['cy'] - f['fy'] * pc[:, 2] / np.where(davanti, x, 1)
        dist = np.linalg.norm(v - t, axis=1)
        dentro = davanti & (u >= 0) & (u < f['w'] - 1) & (w >= 0) & (w < f['h'] - 1) & (dist < max_dist)
        if eliche:
            chiave = (f['w'], f['h'], f['fx'], f['fy'], f['cx'], f['cy'])
            if chiave not in maschere:
                maschere[chiave] = maschera_eliche(*chiave)
            ok = np.nonzero(dentro)[0]
            dentro[ok] = ~maschere[chiave][w[ok].astype(int), u[ok].astype(int)]
        idx = np.nonzero(dentro)[0]
        if len(idx) == 0:
            continue
        direz = (v[idx] - t) / dist[idx, None]
        raggi = np.hstack([np.repeat(t[None, :], len(idx), 0), direz]).astype(np.float32)
        hit = scena.cast_rays(o3d.core.Tensor(raggi))['t_hit'].numpy()
        # Tolleranza che cresce con la distanza: superfici viste lontane e di
        # sbieco (il pavimento a 3-6 m) venivano scartate per i piccoli dossi
        # della superficie ricostruita lungo il raggio (+0.7% di mappa colorata).
        visibili = hit >= dist[idx] - (0.04 + 0.03 * dist[idx])
        idx = idx[visibili]
        col = img[w[idx].astype(int), u[idx].astype(int)] / 255.0
        pw = 1.0 / np.maximum(dist[idx], 0.3) ** 2
        np.add.at(somma, idx, col * pw[:, None])
        np.add.at(peso, idx, pw)
    colori = np.full((len(v), 3), 0.6)
    visti = peso > 0
    colori[visti] = somma[visti] / peso[visti, None]
    return colori, int(visti.sum())


def anteprime_opengl(mesh, centro, viste, cartella, larghezza=1600, altezza=1200):
    """Anteprime col visualizzatore OpenGL classico di Open3D (lo stesso della
    finestra di mostra_mappa.py), in una finestra nascosta. Sul Jetson il renderer
    fuori schermo (Filament su Vulkan) restituiva immagini tutte nere in ogni run,
    anche con la mesh ridotta (17 ottobre 2026); questo disegna la mesh intera.
    None se non c'e' un display o se le immagini escono comunque vuote: si ripiega
    sul renderer fuori schermo."""
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        return None
    vis = o3d.visualization.Visualizer()
    if not vis.create_window(width=larghezza, height=altezza, visible=False):
        return None
    try:
        opt = vis.get_render_option()
        opt.background_color = np.array([1.0, 1.0, 1.0])
        opt.mesh_show_back_face = True
        vis.add_geometry(mesh)
        ctr = vis.get_view_control()
        uscite = []
        for nome, (occhio, su, zoom) in viste.items():
            davanti = np.asarray(occhio, float) - np.asarray(centro, float)
            ctr.set_lookat(centro)
            ctr.set_front(davanti / np.linalg.norm(davanti))
            ctr.set_up(su)
            ctr.set_zoom(zoom)
            for _ in range(3):
                vis.poll_events()
                vis.update_renderer()
            img = np.asarray(vis.capture_screen_float_buffer(True))
            # Vuota = tutta dello sfondo bianco (o nera): il disegno non e' riuscito.
            if img.size == 0 or img.min() > 0.99 or img.max() < 0.01:
                return None
            p = os.path.join(cartella, f'anteprima_{nome}.png')
            o3d.io.write_image(p, o3d.geometry.Image((img * 255).astype(np.uint8)))
            uscite.append(p)
        return uscite
    finally:
        vis.destroy_window()


def anteprime(mesh, cartella):
    bb = mesh.get_axis_aligned_bounding_box()
    centro = bb.get_center()
    est = bb.get_extent()
    # Il soffitto coprirebbe tutto nella vista dall'alto: lo si toglie per l'anteprima.
    v = np.asarray(mesh.vertices)
    senza_soffitto = o3d.geometry.TriangleMesh(mesh)
    senza_soffitto.remove_vertices_by_mask(v[:, 2] > bb.max_bound[2] - 0.35)
    senza_soffitto.compute_vertex_normals()
    # (occhio, alto dell'immagine, zoom del visualizzatore OpenGL)
    viste = {
        'alto': ([centro[0], centro[1], centro[2] + max(est[0], est[1]) * 1.1], [0, 1, 0], 0.62),
        'prospettiva': ([centro[0] - est[0] * 0.7, centro[1] - est[1] * 0.9,
                         centro[2] + max(est[0], est[1]) * 0.6], [0, 0, 1], 0.58),
    }
    arm = platform.machine() in ('aarch64', 'arm64')
    if arm:
        uscite = anteprime_opengl(senza_soffitto, centro, viste, cartella)
        if uscite:
            return uscite
    # Jetson: il renderer fuori schermo (Filament su Vulkan) restituisce immagini
    # tutte nere con la mesh intera (1.2 M triangoli) e oltre 1280x720 circa: per
    # l'anteprima si riduce la mesh e la risoluzione (vedi video_demo.py). Sul
    # Jetson resta solo come riserva del visualizzatore OpenGL qui sopra.
    if arm and len(senza_soffitto.triangles) > 250000:
        senza_soffitto = senza_soffitto.simplify_quadric_decimation(250000)
        senza_soffitto.compute_vertex_normals()
    r = o3d.visualization.rendering.OffscreenRenderer(*((1280, 960) if arm else (1600, 1200)))
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = 'defaultLit'
    r.scene.set_background([1, 1, 1, 1])
    r.scene.add_geometry('mappa', senza_soffitto, mat)
    uscite = []
    # Il primo fotogramma del renderer fuori schermo esce spesso nero (il motore
    # non ha ancora caricato la geometria sulla GPU): se ne butta uno a vuoto.
    r.setup_camera(55.0, centro, viste['alto'][0], viste['alto'][1])
    r.render_to_image()
    for nome, (occhio, su, _) in viste.items():
        r.setup_camera(55.0, centro, occhio, su)
        p = os.path.join(cartella, f'anteprima_{nome}.png')
        for _ in range(3):   # un fotogramma tutto nero si riprova (Jetson)
            img = r.render_to_image()
            if np.asarray(img).max() > 0:
                break
        o3d.io.write_image(p, img)
        uscite.append(p)
    return uscite


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cartella', nargs='?', default='/tmp/c2x_mappa_realistica')
    ap.add_argument('--depth', type=int, default=9,
                    help='profondita\' di Poisson: 9 = celle di ~4 cm su una scena di 20 m')
    ap.add_argument('--max_triangoli', type=int, default=1500000)
    # 1 (era 2): le pareti viste solo da lontano o di sbieco hanno celle da 2 cm
    # colpite UNA volta sola, e scartarle bucava proprio le parti alte delle
    # pareti. Con FAST-LIO a 0.2 m e le scansioni che non si perdono piu' il
    # rumore isolato e' sparito: su 3 run (V15) copertura delle superfici
    # visibili 98.7-99.7% -> 99.97-100%, precisione invariata al 100% entro 15 cm
    # e leggermente migliore entro 5 cm.
    ap.add_argument('--min_conteggio', type=int, default=1,
                    help='voxel osservati meno volte di cosi\' sono considerati rumore')
    # 10 cm: con 6 cm la superficie fra punti radi (pareti lontane, spaziatura
    # dei raggi > 12 cm) veniva bucata.
    ap.add_argument('--taglio', type=float, default=0.10,
                    help='distanza massima (m) di un vertice dal punto misurato piu\' vicino')
    ap.add_argument('--max_dist', type=float, default=8.0,
                    help='oltre questa distanza un fotogramma non colora (troppo poco dettaglio)')
    # 1000 e non 250: con la fase fotografica una run ne salva ~400, e prenderne
    # 250 a intervalli regolari buttava proprio quelli delle ultime zone.
    ap.add_argument('--max_fotogrammi', type=int, default=1000)
    # Pixel dove passano le pale delle eliche anteriori esclusi dal colore (vedi
    # maschera_eliche); --con_eliche torna al comportamento di prima.
    ap.add_argument('--con_eliche', action='store_true')
    a = ap.parse_args()

    punti, direz = carica_punti(a.cartella, a.min_conteggio)
    print(f'[ricostruisci] {len(punti)} punti lidar a 2 cm', flush=True)
    mesh, pc = superficie(punti, direz, a.depth, a.taglio, a.max_triangoli)
    print(f'[ricostruisci] superficie: {len(mesh.vertices)} vertici, {len(mesh.triangles)} triangoli',
          flush=True)
    colori, visti = colora(mesh, a.cartella, a.max_dist, a.max_fotogrammi, eliche=not a.con_eliche)
    mesh.vertex_colors = o3d.utility.Vector3dVector(colori)
    print(f'[ricostruisci] colore dalle foto per {visti}/{len(colori)} vertici '
          f'({100.0 * visti / max(1, len(colori)):.0f}%)', flush=True)
    o3d.io.write_triangle_mesh(os.path.join(a.cartella, 'mappa_realistica.ply'), mesh)
    nuvola = o3d.geometry.PointCloud(mesh.vertices)
    nuvola.colors = mesh.vertex_colors
    o3d.io.write_point_cloud(os.path.join(a.cartella, 'nuvola_colorata.ply'), nuvola)
    try:
        for p in anteprime(mesh, a.cartella):
            print(f'[ricostruisci] anteprima {p}', flush=True)
    except Exception as e:  # rendering non disponibile: la mesh resta comunque salvata
        print(f'[ricostruisci] anteprime non generate: {e}', flush=True)
    print(f'[ricostruisci] mappa realistica: {os.path.join(a.cartella, "mappa_realistica.ply")}',
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
