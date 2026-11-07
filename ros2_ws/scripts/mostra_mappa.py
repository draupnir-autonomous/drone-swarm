#!/usr/bin/env python3
"""Finestra interattiva sulla mappa realistica 3D (Open3D).

Lanciata da run_multi_exploration.sh appena la mappa e' pronta (MOSTRA_MAPPA=0
la disattiva; le campagne la disattivano sempre). Si puo' anche aprire a mano:

    python3 mostra_mappa.py [cartella o file .ply]

Comandi nella finestra:
    trascina col sinistro   ruota
    trascina col destro / Ctrl+sinistro   sposta
    rotella                 zoom
    C                       mostra / nasconde il soffitto (nascosto all'apertura,
                            altrimenti dall'alto copre tutto)
    R                       torna alla vista iniziale
    Q o Esc                 chiude
"""
import os
import sys

import numpy as np
import open3d as o3d


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2x_run_multi/mappa_realistica_drone1'
    ply = os.path.join(arg, 'mappa_realistica.ply') if os.path.isdir(arg) else arg
    if not os.path.exists(ply):
        print(f'[mostra_mappa] mappa non trovata: {ply}')
        return 1
    completa = o3d.io.read_triangle_mesh(ply)
    completa.compute_vertex_normals()
    v = np.asarray(completa.vertices)
    tetto = completa.get_axis_aligned_bounding_box().max_bound[2]
    senza = o3d.geometry.TriangleMesh(completa)
    senza.remove_vertices_by_mask(v[:, 2] > tetto - 0.35)
    senza.compute_vertex_normals()

    stato = {'soffitto': False}

    def cambia_soffitto(vis):
        vis.clear_geometries()
        stato['soffitto'] = not stato['soffitto']
        vis.add_geometry(completa if stato['soffitto'] else senza, reset_bounding_box=False)
        return False

    def vista_iniziale(vis):
        vis.reset_view_point(True)
        return False

    print('[mostra_mappa] ruota: trascina | sposta: tasto destro | zoom: rotella | '
          'C: soffitto | R: vista iniziale | Q: chiudi', flush=True)
    o3d.visualization.draw_geometries_with_key_callbacks(
        [senza], {ord('C'): cambia_soffitto, ord('R'): vista_iniziale},
        window_name='Mappa 3D dell\'esplorazione', width=1400, height=950)
    return 0


if __name__ == '__main__':
    sys.exit(main())
