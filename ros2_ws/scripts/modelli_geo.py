"""Geometria di COLLISIONE di un modello Gazebo (model.sdf), come nuvola di punti nel
riferimento del modello: mesh .dae/.obj (nodi con matrici, up_axis, unita', scala)
e primitive box/cylinder/sphere. Serve a misurare gli ingombri e a costruire la
mappa vera degli ostacoli di un mondo fatto di modelli."""
import os
import re
import numpy as np
import xml.etree.ElementTree as ET

NSC = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose_matrix(testo):
    v = [float(x) for x in (testo or '0 0 0 0 0 0').split()] + [0.0] * 6
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*v[3:6])
    T[:3, 3] = v[:3]
    return T


def _campiona(V, T):
    """Vertici + baricentri + punti medi dei lati (le facce grandi restano coperte)."""
    if T is None or len(T) == 0:
        return V
    tri = V[T]
    out = [V, tri.mean(1), (tri[:, 0] + tri[:, 1]) / 2, (tri[:, 1] + tri[:, 2]) / 2,
           (tri[:, 0] + tri[:, 2]) / 2]
    # triangoli grandi: griglia baricentrica fitta
    lati = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
    grandi = tri[lati > 0.3]
    if len(grandi):
        n = 12
        for a in range(n + 1):
            for b in range(n + 1 - a):
                c = n - a - b
                out.append((a * grandi[:, 0] + b * grandi[:, 1] + c * grandi[:, 2]) / n)
    return np.vstack(out)


def _matrice_nodo(nodo, M):
    """M composta con le trasformazioni di un nodo COLLADA, nell'ordine in cui compaiono."""
    for el in list(nodo):
        tag = el.tag.split('}')[-1]
        T = np.eye(4)
        if tag == 'matrix':
            T = np.array(el.text.split(), float).reshape(4, 4)
        elif tag == 'translate':
            T[:3, 3] = [float(x) for x in el.text.split()]
        elif tag == 'rotate':
            ax = np.array([float(x) for x in el.text.split()[:3]]); ang = np.radians(float(el.text.split()[3]))
            ax = ax / max(np.linalg.norm(ax), 1e-12)
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            T[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
        elif tag == 'scale':
            T[:3, :3] = np.diag([float(x) for x in el.text.split()])
        else:
            continue
        M = M @ T
    return M


def triangoli_dae(f):
    """Tutti i triangoli di una mesh .dae (vertici V Nx3, indici T Mx3) nel riferimento
    del modello: nodi con matrici, unita', up_axis. Serve a costruire geometrie
    derivate esattamente dalla collisione (vedi make_parco.chiome_per_lidar)."""
    t = ET.parse(f)
    unit = t.find('.//c:asset/c:unit', NSC)
    metri = float(unit.get('meter')) if unit is not None and unit.get('meter') else 1.0
    up = t.find('.//c:asset/c:up_axis', NSC)
    U = np.eye(4)
    if up is not None and up.text.strip() == 'Y_UP':
        U[:3, :3] = rpy_matrix(np.pi / 2, 0, 0)
    geos = {g.get('id'): g for g in t.findall('.//c:geometry', NSC)}
    Vs, Ts = [], []

    def visita(nodo, M):
        M = _matrice_nodo(nodo, M)
        for ig in nodo.findall('c:instance_geometry', NSC):
            g = geos.get(ig.get('url')[1:])
            if g is None:
                continue
            for mesh in g.findall('c:mesh', NSC):
                srcs = {s.get('id'): s for s in mesh.findall('c:source', NSC)}
                pid = mesh.find('c:vertices/c:input[@semantic="POSITION"]', NSC).get('source')[1:]
                P = np.array(srcs[pid].find('c:float_array', NSC).text.split(), float).reshape(-1, 3)
                base = sum(len(v) for v in Vs)
                Vs.append((U @ M @ np.c_[P, np.ones(len(P))].T).T[:, :3] * metri)
                for tri in mesh.findall('c:triangles', NSC) + mesh.findall('c:polylist', NSC):
                    inputs = tri.findall('c:input', NSC)
                    stride = max(int(i.get('offset')) for i in inputs) + 1
                    voff = [int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX'][0]
                    p = tri.find('c:p', NSC)
                    if p is None or not p.text:
                        continue
                    if tri.tag.endswith('polylist'):
                        vc = np.array(tri.find('c:vcount', NSC).text.split(), int)
                        if not (vc == 3).all():
                            raise ValueError(f'{f}: poligoni non triangolari')
                    idx = np.array(p.text.split(), int).reshape(-1, stride)[:, voff]
                    Ts.append(idx.reshape(-1, 3) + base)
        for figlio in nodo.findall('c:node', NSC):
            visita(figlio, M)

    for vs in t.findall('.//c:library_visual_scenes/c:visual_scene', NSC):
        for nodo in vs.findall('c:node', NSC):
            visita(nodo, np.eye(4))
    return np.vstack(Vs), np.vstack(Ts)


def carica_dae(f):
    t = ET.parse(f)
    unit = t.find('.//c:asset/c:unit', NSC)
    metri = float(unit.get('meter')) if unit is not None and unit.get('meter') else 1.0
    up = t.find('.//c:asset/c:up_axis', NSC)
    U = np.eye(4)
    if up is not None and up.text.strip() == 'Y_UP':
        U[:3, :3] = rpy_matrix(np.pi / 2, 0, 0)
    geos = {g.get('id'): g for g in t.findall('.//c:geometry', NSC)}

    def geom(g):
        Vs, Ts = [], []
        for mesh in g.findall('c:mesh', NSC):
            srcs = {s.get('id'): s for s in mesh.findall('c:source', NSC)}
            vert = mesh.find('c:vertices', NSC)
            pid = vert.find('c:input[@semantic="POSITION"]', NSC).get('source')[1:]
            P = np.array(srcs[pid].find('c:float_array', NSC).text.split(), float).reshape(-1, 3)
            base = 0
            for tri in mesh.findall('c:triangles', NSC) + mesh.findall('c:polylist', NSC):
                inputs = tri.findall('c:input', NSC)
                stride = max(int(i.get('offset')) for i in inputs) + 1
                voff = [int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX'][0]
                p = tri.find('c:p', NSC)
                if p is None or not p.text:
                    continue
                idx = np.array(p.text.split(), int).reshape(-1, stride)[:, voff]
                if tri.tag.endswith('polylist'):
                    vc = np.array(tri.find('c:vcount', NSC).text.split(), int)
                    if not (vc == 3).all():
                        continue
                Ts.append(idx.reshape(-1, 3))
            Vs.append(P)
        if not Vs:
            return np.zeros((0, 3))
        V = Vs[0] if len(Vs) == 1 else np.vstack(Vs)
        T = np.vstack(Ts) if Ts and len(Vs) == 1 else None
        return _campiona(V, T)

    pts = []

    def visita(nodo, M):
        M = _matrice_nodo(nodo, M)
        for ig in nodo.findall('c:instance_geometry', NSC):
            g = geos.get(ig.get('url')[1:])
            if g is not None:
                P = geom(g)
                if len(P):
                    pts.append((M @ np.c_[P, np.ones(len(P))].T).T[:, :3])
        for figlio in nodo.findall('c:node', NSC):
            visita(figlio, M)

    for vs in t.findall('.//c:library_visual_scenes/c:visual_scene', NSC):
        for nodo in vs.findall('c:node', NSC):
            visita(nodo, np.eye(4))
    if not pts:
        return np.zeros((0, 3))
    P = np.vstack(pts) * metri
    return (U[:3, :3] @ P.T).T


def triangoli_obj(f):
    """Tutti i triangoli di una mesh .obj (V Nx3, T Mx3), poligoni divisi a ventaglio."""
    V, F = [], []
    for riga in open(f, errors='ignore'):
        if riga.startswith('v '):
            V.append([float(x) for x in riga.split()[1:4]])
        elif riga.startswith('f '):
            idx = [int(x.split('/')[0]) for x in riga.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1):
                F.append([idx[0], idx[k], idx[k + 1]])
    return np.array(V, float), np.array(F, int)


def carica_obj(f):
    V, F = [], []
    for riga in open(f, errors='ignore'):
        if riga.startswith('v '):
            V.append([float(x) for x in riga.split()[1:4]])
        elif riga.startswith('f '):
            idx = [int(x.split('/')[0]) for x in riga.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1):
                F.append([idx[0], idx[k], idx[k + 1]])
    V = np.array(V, float)
    return _campiona(V, np.array(F, int) if F else None)


def risolvi_uri(uri, cartella_modello, radici):
    uri = uri.strip()
    if uri.startswith('model://'):
        nome, resto = uri[len('model://'):].split('/', 1)
        for r in radici:
            p = os.path.join(r, nome, resto)
            if os.path.exists(p):
                return p
        return os.path.join(cartella_modello, resto)
    return os.path.join(cartella_modello, uri)


def punti_collisione(cartella_modello, radici):
    """Punti delle collisioni nel riferimento del modello."""
    s = open(os.path.join(cartella_modello, 'model.sdf')).read()
    root = ET.fromstring(s)
    modello = root.find('model')
    Mm = pose_matrix(modello.findtext('pose'))
    out = []
    for link in modello.findall('link'):
        Ml = Mm @ pose_matrix(link.findtext('pose'))
        for col in link.findall('collision'):
            Mc = Ml @ pose_matrix(col.findtext('pose'))
            g = col.find('geometry')
            P = None
            if g.find('mesh') is not None:
                mesh = g.find('mesh')
                f = risolvi_uri(mesh.findtext('uri'), cartella_modello, radici)
                P = carica_dae(f) if f.lower().endswith('.dae') else carica_obj(f)
                sc = mesh.findtext('scale')
                if sc:
                    P = P * np.array([float(x) for x in sc.split()])
            elif g.find('box') is not None:
                sx, sy, sz = [float(x) for x in g.find('box').findtext('size').split()]
                gx, gy, gz = np.meshgrid(*[np.linspace(-a / 2, a / 2, max(2, int(a / 0.05) + 1)) for a in (sx, sy, sz)])
                P = np.c_[gx.ravel(), gy.ravel(), gz.ravel()]
            elif g.find('cylinder') is not None:
                r = float(g.find('cylinder').findtext('radius')); h = float(g.find('cylinder').findtext('length'))
                a, z = np.meshgrid(np.linspace(0, 2 * np.pi, 36), np.linspace(-h / 2, h / 2, max(2, int(h / 0.05))))
                P = np.c_[r * np.cos(a).ravel(), r * np.sin(a).ravel(), z.ravel()]
            elif g.find('sphere') is not None:
                r = float(g.find('sphere').findtext('radius'))
                u, v = np.meshgrid(np.linspace(0, np.pi, 18), np.linspace(0, 2 * np.pi, 36))
                P = r * np.c_[(np.sin(u) * np.cos(v)).ravel(), (np.sin(u) * np.sin(v)).ravel(), np.cos(u).ravel()]
            if P is not None and len(P):
                out.append((Mc @ np.c_[P, np.ones(len(P))].T).T[:, :3])
    return np.vstack(out) if out else np.zeros((0, 3))
