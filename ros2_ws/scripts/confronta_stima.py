#!/usr/bin/env python3
"""Errore della stima di posizione mandata a PX4 (FAST-LIO o FAST-LIVO2) contro la verita'.

Legge dal registro di volo di PX4 (rel_ulog_*.ulg nella cartella della run):
  vehicle_visual_odometry             la stima esterna cosi' come e' arrivata a EKF2
  vehicle_local_position_groundtruth  la posizione vera del simulatore (coordinate mondo)

Le due terne hanno origini e imbardate diverse (la stima parte da zero alla posa di
partenza): si allineano con lo scarto e l'imbardata all'ultimo istante a terra prima del
decollo, senza altri aggiustamenti, cosi' l'errore misurato include la deriva accumulata.

Uso: confronta_stima.py <cartella run> [<cartella run> ...]
"""
import glob
import os
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit('serve pyulog (pip install pyulog)')


def imbardata_q(w, x, y, z):
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def analizza(ulg):
    u = ULog(ulg, ['vehicle_visual_odometry', 'vehicle_local_position_groundtruth'])
    d = {m.name: m.data for m in u.data_list}
    ev, gt = d.get('vehicle_visual_odometry'), d.get('vehicle_local_position_groundtruth')
    if ev is None or gt is None or len(ev['timestamp']) < 20:
        return None
    te, tg = ev['timestamp'] / 1e6, gt['timestamp'] / 1e6
    pe = np.c_[ev['position[0]'], ev['position[1]'], ev['position[2]']].astype(float)
    pg = np.c_[gt['x'], gt['y'], gt['z']].astype(float)
    alt = -pg[:, 2]
    in_aria = np.where(alt > alt[:50].mean() + 0.2)[0]
    if in_aria.size == 0:
        return None
    t_decollo, t_appoggio = tg[in_aria[0]], tg[in_aria[-1]]

    # Allineamento all'ultimo campione a terra.
    i0 = max(np.searchsorted(te, t_decollo) - 1, 0)
    g0 = np.array([np.interp(te[i0], tg, pg[:, k]) for k in range(3)])
    psi_g = np.interp(te[i0], tg, np.unwrap(gt['heading']))
    psi_e = imbardata_q(ev['q[0]'][i0], ev['q[1]'][i0], ev['q[2]'][i0], ev['q[3]'][i0])
    a = psi_g - psi_e
    R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    stima = (pe - pe[i0]) @ R.T + g0
    vera = np.c_[[np.interp(te, tg, pg[:, k]) for k in range(3)]].T

    volo = (te >= t_decollo) & (te <= t_appoggio)
    e = stima[volo] - vera[volo]
    # Ritardo: la stima arriva a PX4 dopo l'istante a cui si riferisce (calcolo e
    # trasporto), e in volo a 1 m/s 0.1 s sono 10 cm. Si cerca lo spostamento nel tempo
    # che rende minimo l'errore orizzontale: l'errore a quel ritardo e' la precisione
    # propria dello stimatore, quello a ritardo zero e' cio' che vede EKF2.
    rit = np.arange(0.0, 0.501, 0.01)
    med = [np.median(np.hypot(*(stima[volo][:, :2] - np.c_[
        np.interp(te[volo] - r, tg, pg[:, 0]), np.interp(te[volo] - r, tg, pg[:, 1])]).T))
        for r in rit]
    k = int(np.argmin(med))
    e_rit = stima[volo] - np.c_[[np.interp(te[volo] - rit[k], tg, pg[:, j]) for j in range(3)]].T
    exy_rit = np.hypot(e_rit[:, 0], e_rit[:, 1])
    exy, ez = np.hypot(e[:, 0], e[:, 1]), np.abs(e[:, 2])
    dt = np.diff(te[volo])
    r = dict(volo_s=t_appoggio - t_decollo, campioni=int(volo.sum()),
             hz=1.0 / np.median(dt), buco_max=float(dt.max()),
             xy_med=np.median(exy), xy_p95=np.percentile(exy, 95), xy_max=exy.max(),
             z_med=np.median(ez), z_p95=np.percentile(ez, 95), z_max=ez.max(),
             xy_fine=exy[-1], z_fine=ez[-1],
             ritardo=rit[k], xyr_med=np.median(exy_rit), xyr_p95=np.percentile(exy_rit, 95))
    return r


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    print("errori in metri; xy, z: stima contro verita' allo stesso istante (cio' che vede EKF2);")
    print("rit.: ritardo della stima; xy rit.: errore orizzontale tolto il ritardo; fine: all'appoggio")
    print(f'{"run":<36} {"volo":>5} {"Hz":>5} {"buco":>5} | {"xy med":>6} {"p95":>5} {"max":>5} '
          f'{"fine":>5} | {"rit.":>5} {"xy rit. med":>11} {"p95":>5} | {"z med":>5} {"p95":>5} '
          f'{"max":>5}')
    for cartella in sys.argv[1:]:
        for ulg in sorted(glob.glob(os.path.join(cartella, '*.ulg'))):
            r = analizza(ulg)
            nome = os.path.basename(os.path.normpath(cartella))
            if r is None:
                print(f'{nome:<40} senza stima esterna o senza volo')
                continue
            print(f'{nome:<36} {r["volo_s"]:4.0f}s {r["hz"]:5.1f} {r["buco_max"]:4.2f}s | '
                  f'{r["xy_med"]:6.3f} {r["xy_p95"]:5.3f} {r["xy_max"]:5.2f} {r["xy_fine"]:5.2f} | '
                  f'{r["ritardo"]:4.2f}s {r["xyr_med"]:11.3f} {r["xyr_p95"]:5.3f} | '
                  f'{r["z_med"]:5.3f} {r["z_p95"]:5.3f} {r["z_max"]:5.2f}')


if __name__ == '__main__':
    main()
