#!/usr/bin/env python3
"""Trova i BLOCCHI di una run: intervalli in cui il drone e' quasi fermo mentre
dovrebbe esplorare, e dice cosa stava facendo il sistema in quei secondi.

Un blocco e' un tratto di almeno --durata secondi in cui la riga [Traccia]
dell'FSM (una al secondo) riporta velocita' sotto --vel con lo stato in
PLAN_TRAJ / PUB_TRAJ / EXEC_TRAJ / IDLE, fuori dal rientro. Per ogni blocco si
contano i messaggi che compaiono in quell'intervallo e se ne ricava la causa
dominante.

Uso: analisi_blocchi.py <exploration.log> [...] [--durata 10] [--vel 0.15]
"""
import argparse
import collections
import re
import sys

# Firme delle cause note, nell'ordine in cui si controllano.
CAUSE = [
    ('ripianificazione per collisione', r'Replan: collision detected'),
    ('pianificazione fallita', r'Plan fail'),
    ('frontiera irraggiungibile dalla cella', r'No reachable frontier found'),
    ('nessuna cella assegnata', r'No grid|senza griglie assegnate'),
    ('filtro di sicurezza', r'\[Filtro\]'),
    ('fermata di prossimita\'', r'PROXIMITY STOP'),
    ('dietro la traiettoria', r'behind its trajectory'),
    ('odometria scaduta', r'Odometria scaduta'),
    ('traiettoria completata, ripianifico', r'Replan: traj fully executed'),
    ('ripianificazione periodica', r'Replan: periodic call'),
]


def leggi(path):
    righe = []
    for l in open(path, errors='replace'):
        m = re.search(r'\[(1[0-9]{9}\.\d+)\]', l)
        if m:
            righe.append((float(m.group(1)), l))
    return righe


def blocchi(righe, durata, vel_max):
    tracce = []
    for t, l in righe:
        m = re.search(r'\[Traccia\] stato=(\S+) vel=([\d.inf]+) frontiere=(-?\d+) celle=\d+ rientro=(\d)', l)
        if m:
            v = float(m.group(2)) if m.group(2) != 'inf' else 0.0
            tracce.append((t, m.group(1), v, int(m.group(3)), m.group(4) == '1'))
    out, inizio, ultimo = [], None, None
    for t, stato, v, nf, rientro in tracce:
        fermo = (v < vel_max and stato in ('PLAN_TRAJ', 'PUB_TRAJ', 'EXEC_TRAJ', 'IDLE')
                 and not rientro)
        if fermo:
            if inizio is None:
                inizio = (t, nf)
            ultimo = (t, nf)
        else:
            if inizio is not None and ultimo[0] - inizio[0] >= durata:
                out.append((inizio[0], ultimo[0], inizio[1], ultimo[1]))
            inizio = None
    if inizio is not None and ultimo[0] - inizio[0] >= durata:
        out.append((inizio[0], ultimo[0], inizio[1], ultimo[1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='+')
    ap.add_argument('--durata', type=float, default=10.0)
    ap.add_argument('--vel', type=float, default=0.15)
    a = ap.parse_args()

    totale = collections.Counter()
    for path in a.log:
        righe = leggi(path)
        if not righe:
            continue
        t0 = righe[0][0]
        bl = blocchi(righe, a.durata, a.vel)
        tot_s = sum(b[1] - b[0] for b in bl)
        print(f'== {path}: {len(bl)} blocchi, {tot_s:.0f} s fermo')
        for b0, b1, nf0, nf1 in bl:
            conteggi = collections.Counter()
            for t, l in righe:
                if b0 <= t <= b1:
                    for nome, firma in CAUSE:
                        if re.search(firma, l):
                            conteggi[nome] += 1
            dominante = conteggi.most_common(1)[0][0] if conteggi else 'nessun messaggio (in attesa)'
            totale[dominante] += b1 - b0
            dett = ', '.join(f'{k} {v}' for k, v in conteggi.most_common(4))
            print(f'   {b0 - t0:6.0f}-{b1 - t0:4.0f} s ({b1 - b0:4.0f} s)  frontiere {nf0}->{nf1}  '
                  f'causa: {dominante}  [{dett}]')
    if totale:
        print('\nSECONDI DI BLOCCO PER CAUSA DOMINANTE (tutti i log):')
        for k, v in totale.most_common():
            print(f'   {v:6.0f} s  {k}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
