#!/usr/bin/env python3
"""Geometria di COLLISIONE di un mondo Gazebo (.sdf) qualsiasi, come triangoli in
coordinate mondo.

Legge i modelli scritti nel mondo e quelli inclusi (model://nome cercato nei percorsi
delle risorse, URL di Fuel se gia' scaricati in ~/.gz/fuel, percorsi relativi al
mondo), con le pose di include, modello, link e collisione, e le geometrie box,
cylinder, sphere, capsule, ellipsoid, plane e mesh (.dae, .obj, .stl). Il piano di
terra resta a parte (non e' un ostacolo).

Serve a chi deve conoscere un mondo SENZA saperne niente in anticipo:
prepara_mondo.py (zona da esplorare, punto di partenza libero, recinto per i mondi
aperti) e analisi_campagna.py (distanza vera del drone dagli ostacoli).

Limiti dichiarati (si avvisa, non si indovina): pose con relative_to diverso dal
genitore, heightmap e mesh in formati diversi da quelli sopra non vengono letti.

Uso da riga di comando, per controllo:
  python3 geometria_mondo.py <mondo.sdf>      riassunto di pezzi, triangoli e ingombri
"""
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
from modelli_geo import pose_matrix, triangoli_dae, triangoli_obj  # noqa: E402

WS = os.path.dirname(QUI)
PROGETTO = os.path.dirname(WS)


def radici_risorse(mondo_sdf, extra=()):
    """Cartelle in cui cercare i model://: accanto al mondo, i modelli del progetto e
    di PX4, GZ_SIM_RESOURCE_PATH, la cache di Fuel."""
    d = os.path.dirname(os.path.abspath(mondo_sdf))
    radici = [d, os.path.join(d, 'models'), os.path.join(d, '..', 'models'), *extra,
              os.path.join(WS, 'src', 'c2x_worlds', 'models_parco'),
              os.path.join(WS, 'src', 'c2x_description', 'models'),
              os.path.join(PROGETTO, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'models')]
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'SDF_PATH'):
        radici += [p for p in os.environ.get(var, '').split(':') if p]
    visti, out = set(), []
    for r in radici:
        r = os.path.realpath(r)
        if os.path.isdir(r) and r not in visti:
            visti.add(r)
            out.append(r)
    return out


def _cartella_fuel(uri):
    """https://<server>/1.0/<owner>/models/<nome>[/<versione>] -> cartella nella cache."""
    parti = uri.split('://', 1)[1].split('/')
    if len(parti) < 5 or parti[3].lower() != 'models':
        return None
    server, owner, nome = parti[0], parti[2], parti[4]
    base = os.path.join(os.path.expanduser('~/.gz/fuel'), server, owner.lower(), 'models', nome.lower())
    if not os.path.isdir(base):
        base = os.path.join(os.path.expanduser('~/.gz/fuel'), server, owner, 'models', nome)
    if not os.path.isdir(base):
        return None
    versioni = sorted((v for v in os.listdir(base) if v.isdigit()), key=int)
    return os.path.join(base, versioni[-1]) if versioni else None


def cartella_modello(uri, base, radici):
    uri = uri.strip()
    if uri.startswith('model://'):
        nome = uri[len('model://'):].split('/')[0]
        for r in radici:
            p = os.path.join(r, nome)
            if os.path.isdir(p):
                return p
        return None
    if uri.startswith('http://') or uri.startswith('https://'):
        return _cartella_fuel(uri)
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    p = uri if os.path.isabs(uri) else os.path.join(base, uri)
    return p if os.path.isdir(p) else None


def file_sdf_modello(cartella):
    """Il file .sdf di un modello: quello di versione piu' alta in model.config."""
    cfg = os.path.join(cartella, 'model.config')
    if os.path.exists(cfg):
        try:
            voci = [(e.get('version') or '0', (e.text or '').strip())
                    for e in ET.parse(cfg).getroot().findall('sdf')]
            voci = [v for v in voci if v[1]]
            if voci:
                voci.sort(key=lambda v: [int(x) for x in v[0].split('.') if x.isdigit()])
                return os.path.join(cartella, voci[-1][1])
        except ET.ParseError:
            pass
    return os.path.join(cartella, 'model.sdf')


def risolvi_mesh(uri, cartella, radici):
    uri = uri.strip()
    if uri.startswith('model://'):
        nome, resto = uri[len('model://'):].split('/', 1)
        for r in radici:
            p = os.path.join(r, nome, resto)
            if os.path.exists(p):
                return p
        return None
    if uri.startswith('http://') or uri.startswith('https://'):
        # https://<server>/1.0/<owner>/models/<nome>/<versione?>/files/<percorso>
        if '/files/' in uri:
            c = _cartella_fuel(uri.split('/files/')[0])
            if c:
                p = os.path.join(c, uri.split('/files/', 1)[1])
                return p if os.path.exists(p) else None
        return None
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    p = uri if os.path.isabs(uri) else os.path.join(cartella, uri)
    return p if os.path.exists(p) else None


def triangoli_stl(f):
    dati = open(f, 'rb').read()
    if dati[:5].lower() == b'solid' and b'facet' in dati[:1000]:
        V = [[float(x) for x in r.split()[1:4]] for r in dati.decode(errors='ignore').splitlines()
             if r.strip().startswith('vertex')]
        V = np.array(V, float)
    else:
        n = struct.unpack('<I', dati[80:84])[0]
        a = np.frombuffer(dati[84:84 + 50 * n], dtype=np.uint8).reshape(n, 50)
        V = a[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)
    return V, np.arange(len(V)).reshape(-1, 3)


def _box(sx, sy, sz):
    V = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def _cilindro(r, h, n=24):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    giu = np.c_[r * np.cos(a), r * np.sin(a), np.full(n, -h / 2)]
    su = np.c_[r * np.cos(a), r * np.sin(a), np.full(n, h / 2)]
    V = np.vstack([giu, su, [[0, 0, -h / 2], [0, 0, h / 2]]])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return V, np.array(F)


def _ellissoide(rx, ry, rz, n=12):
    u, v = np.meshgrid(np.linspace(0, np.pi, n + 1), np.linspace(0, 2 * np.pi, 2 * n, endpoint=False))
    V = np.c_[(rx * np.sin(u) * np.cos(v)).ravel(), (ry * np.sin(u) * np.sin(v)).ravel(), (rz * np.cos(u)).ravel()]
    idx = np.arange(V.shape[0]).reshape(2 * n, n + 1)
    F = []
    for i in range(2 * n):
        k = (i + 1) % (2 * n)
        for j in range(n):
            F += [[idx[i, j], idx[k, j], idx[k, j + 1]], [idx[i, j], idx[k, j + 1], idx[i, j + 1]]]
    return V, np.array(F)


class Geometria:
    """pezzi: lista di (nome, V Nx3 mondo, F Mx3); terra: quote dei piani orizzontali."""

    def __init__(self):
        self.pezzi = []
        self.terra = []
        self.avvisi = []

    def avvisa(self, testo):
        if testo not in self.avvisi:
            self.avvisi.append(testo)

    def tutti(self):
        if not self.pezzi:
            return np.zeros((0, 3)), np.zeros((0, 3), int)
        Vs, Fs, base = [], [], 0
        for _, V, F in self.pezzi:
            Vs.append(V)
            Fs.append(F + base)
            base += len(V)
        return np.vstack(Vs), np.vstack(Fs)

    def scena(self, con_terra=False):
        """RaycastingScene di Open3D con tutti gli ostacoli (e i piani di terra se
        con_terra: per le distanze no, il drone appoggiato distera' 0 dal terreno)."""
        import open3d as o3d
        s = o3d.t.geometry.RaycastingScene()
        V, F = self.tutti()
        if len(F):
            s.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
        for z in (self.terra if con_terra else []):
            Vp = np.array([[-500, -500, z], [500, -500, z], [500, 500, z], [-500, 500, z]], np.float32)
            s.add_triangles(o3d.core.Tensor(Vp), o3d.core.Tensor(np.array([[0, 1, 2], [0, 2, 3]], np.uint32)))
        return s

    def distanza(self, X):
        """Distanza vera di ogni punto Nx3 dall'ostacolo piu' vicino (terra esclusa)."""
        import open3d as o3d
        V, F = self.tutti()
        if not len(F):
            return np.full(len(X), np.inf)
        s = o3d.t.geometry.RaycastingScene()
        s.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
        return s.compute_distance(o3d.core.Tensor(np.asarray(X, np.float32))).numpy()


def _geometria(g, M, nome, cartella, radici, geo):
    """Aggiunge a geo la geometria g (elemento <geometry>) nella posa M."""
    for el in g:
        tipo = el.tag
        V = F = None
        if tipo == 'box':
            V, F = _box(*[float(v) for v in el.findtext('size', '1 1 1').split()])
        elif tipo == 'cylinder':
            V, F = _cilindro(float(el.findtext('radius', '1')), float(el.findtext('length', '1')))
        elif tipo == 'sphere':
            r = float(el.findtext('radius', '1'))
            V, F = _ellissoide(r, r, r)
        elif tipo == 'ellipsoid':
            V, F = _ellissoide(*[float(v) for v in el.findtext('radii', '1 1 1').split()])
        elif tipo == 'capsule':
            r, h = float(el.findtext('radius', '0.5')), float(el.findtext('length', '1'))
            V, F = _cilindro(r, h + 2 * r)   # per eccesso: cilindro che contiene le calotte
        elif tipo == 'plane':
            n = np.array([float(v) for v in el.findtext('normal', '0 0 1').split()])
            dim = [float(v) for v in el.findtext('size', '1 1').split()]
            n_mondo = M[:3, :3] @ (n / np.linalg.norm(n))
            if n_mondo[2] > 0.99:
                # Piano orizzontale: per la fisica di Gazebo e' infinito qualunque sia
                # <size> (walls.sdf ha un piano di terra "1 x 1"). E' terra, non ostacolo.
                geo.terra.append(float(M[2, 3]))
                continue
            V = np.array([[-dim[0] / 2, -dim[1] / 2, 0], [dim[0] / 2, -dim[1] / 2, 0],
                          [dim[0] / 2, dim[1] / 2, 0], [-dim[0] / 2, dim[1] / 2, 0]])
            F = np.array([[0, 1, 2], [0, 2, 3]])
            # Il piano e' definito dalla normale: si ruota z sulla normale data.
            z = np.array([0, 0, 1.0]); nn = n / np.linalg.norm(n)
            ax = np.cross(z, nn)
            if np.linalg.norm(ax) > 1e-9:
                ang = np.arccos(np.clip(z @ nn, -1, 1)); ax /= np.linalg.norm(ax)
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                V = V @ (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K).T
        elif tipo == 'mesh':
            f = risolvi_mesh(el.findtext('uri', ''), cartella, radici)
            if f is None:
                geo.avvisa(f'mesh non trovata: {el.findtext("uri", "").strip()} ({nome})')
                continue
            est = os.path.splitext(f)[1].lower()
            try:
                if est == '.dae':
                    V, F = triangoli_dae(f)
                elif est == '.obj':
                    V, F = triangoli_obj(f)
                elif est == '.stl':
                    V, F = triangoli_stl(f)
                else:
                    geo.avvisa(f'formato di mesh non letto: {os.path.basename(f)} ({nome})')
                    continue
            except Exception as e:   # noqa: BLE001 - una mesh rotta non deve fermare tutto
                geo.avvisa(f'mesh illeggibile {os.path.basename(f)} ({nome}): {e}')
                continue
            if el.findtext('scale'):
                V = V * np.array([float(v) for v in el.findtext('scale').split()])
        elif tipo in ('heightmap', 'polyline'):
            geo.avvisa(f'geometria {tipo} non letta ({nome})')
            continue
        else:
            continue
        if V is None or not len(F):
            continue
        geo.pezzi.append((nome, (M @ np.c_[V, np.ones(len(V))].T).T[:, :3], np.asarray(F, int)))


def _posa(el):
    p = el.find('pose')
    if p is not None and p.get('relative_to') not in (None, '', '__model__'):
        return pose_matrix(p.text), p.get('relative_to')
    return pose_matrix(el.findtext('pose')), None


def _modello(m, M, nome, cartella, radici, geo, profondita=0):
    if profondita > 8:
        return
    Pm, rel = _posa(m)
    if rel:
        geo.avvisa(f'posa relative_to="{rel}" trattata come relativa al genitore ({nome})')
    Mm = M @ Pm
    for link in m.findall('link'):
        Pl, rel = _posa(link)
        if rel:
            geo.avvisa(f'posa relative_to="{rel}" trattata come relativa al modello ({nome})')
        Ml = Mm @ Pl
        for col in link.findall('collision'):
            Pc, _ = _posa(col)
            g = col.find('geometry')
            if g is not None:
                _geometria(g, Ml @ Pc, nome, cartella, radici, geo)
    for sotto in m.findall('model'):
        _modello(sotto, Mm, f'{nome}/{sotto.get("name")}', cartella, radici, geo, profondita + 1)
    for inc in m.findall('include'):
        _include(inc, Mm, nome, cartella, radici, geo, profondita + 1)


def _include(inc, M, nome_padre, base, radici, geo, profondita=0):
    uri = inc.findtext('uri', '').strip()
    c = cartella_modello(uri, base, radici)
    nome = inc.findtext('name') or uri.rstrip('/').split('/')[-1]
    if nome_padre:
        nome = f'{nome_padre}/{nome}'
    if c is None:
        geo.avvisa(f'modello non trovato: {uri} ({nome})')
        return
    f = file_sdf_modello(c)
    try:
        radice = ET.parse(f).getroot()
    except (ET.ParseError, OSError) as e:
        geo.avvisa(f'modello illeggibile {f}: {e}')
        return
    m = radice.find('model')
    if m is None:
        geo.avvisa(f'nessun <model> in {f}')
        return
    Pi, _ = _posa(inc)
    # La posa dell'include SOSTITUISCE quella scritta nel modello.
    if inc.find('pose') is not None:
        m = _senza_posa(m)
    _modello(m, M @ Pi, nome, c, radici, geo, profondita)


def _senza_posa(m):
    import copy
    m = copy.deepcopy(m)
    p = m.find('pose')
    if p is not None:
        m.remove(p)
    return m


def carica(mondo_sdf, radici=None):
    """Geometria di collisione del mondo. radici: cartelle dei modelli (default: vedi
    radici_risorse)."""
    radici = radici or radici_risorse(mondo_sdf)
    w = ET.parse(mondo_sdf).getroot().find('world')
    geo = Geometria()
    base = os.path.dirname(os.path.abspath(mondo_sdf))
    for m in w.findall('model'):
        _modello(m, np.eye(4), m.get('name'), base, radici, geo)
    for inc in w.findall('include'):
        _include(inc, np.eye(4), '', base, radici, geo)
    if w.find('.//heightmap') is not None:
        geo.avvisa('il mondo contiene una heightmap: il terreno non e\' piano e non viene letto')
    return geo


def main():
    geo = carica(sys.argv[1])
    V, F = geo.tutti()
    print(f'{len(geo.pezzi)} pezzi, {len(F)} triangoli; piani di terra a z = {geo.terra}')
    if len(V):
        print(f'ingombro ostacoli: x {V[:, 0].min():.2f}..{V[:, 0].max():.2f}  '
              f'y {V[:, 1].min():.2f}..{V[:, 1].max():.2f}  z {V[:, 2].min():.2f}..{V[:, 2].max():.2f}')
    for a in geo.avvisi:
        print('AVVISO:', a)


if __name__ == '__main__':
    main()
