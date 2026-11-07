#!/usr/bin/env python3
"""FASI DELLA MISSIONE per drone, in secondi SIMULATI (sciame, ottobre 2026).

Dal registro dell'esplorazione (rel_expl_1.log di una run archiviata, o
/tmp/c2x_run_multi/exploration.log): le righe [Traccia] dell'FSM portano sia l'orologio di
parete del logger sia il tempo dello stack (t_sim), quindi ogni evento del registro si
riporta in tempo simulato interpolando fra le due. Per ogni drone:

  pronto    decollo e giro iniziale conclusi
  via       trigger accettato (WAIT_TRIGGER -> PLAN_TRAJ)
  esplor.   "has finished exploration" (frontiere finite)
  visita    inizio del rientro (fine della visita / delle foto)
  casa      arrivato sopra la partenza, atterraggio chiesto

Uso: fasi_missione.py <cartella_run | exploration.log> [...]
"""
import os
import re
import sys

import numpy as np

RIGA = re.compile(r'\[(\d+\.\d+)\] \[drone_(\d+)\.exploration_node\]: (.*)$')
EVENTI = [
    ('pronto', re.compile(r'decollo e giro iniziale conclusi')),
    ('via', re.compile(r'from WAIT_TRIGGER to PLAN_TRAJ')),
    ('esplor.', re.compile(r'has finished exploration')),
    ('visita', re.compile(r'\[Rientro\] Drone \d+ torna alla partenza|\[Rientro\] Drone \d+ gia\' alla partenza')),
    ('casa', re.compile(r'\[Rientro\] Drone \d+ (arrivato|tempo scaduto|gia\' alla partenza)')),
]


def analizza(percorso):
    log = percorso
    if os.path.isdir(percorso):
        log = os.path.join(percorso, 'rel_expl_1.log')
    parete, sim = [], []
    eventi = {}
    for riga in open(log, errors='replace'):
        m = RIGA.search(riga)
        if not m:
            continue
        tp, d, testo = float(m.group(1)), int(m.group(2)), m.group(3)
        ms = re.search(r't_sim=([\d.]+)', testo)
        if ms:
            parete.append(tp)
            sim.append(float(ms.group(1)))
        for nome, rx in EVENTI:
            if rx.search(testo):
                eventi.setdefault(d, {}).setdefault(nome, tp)
    if len(parete) < 2:
        print(f'{percorso}: nessuna riga [Traccia] con t_sim (codice precedente)')
        return
    ordine = np.argsort(parete)
    parete, sim = np.array(parete)[ordine], np.array(sim)[ordine]

    def a_sim(tp):
        return float(np.interp(tp, parete, sim))
    print(f'== {percorso}')
    t0 = min(a_sim(e['via']) for e in eventi.values() if 'via' in e) if eventi else 0.0
    print(f'   {"drone":>5} ' + ' '.join(f'{n:>8}' for n, _ in EVENTI) + '   (s simulati dal primo via)')
    for d in sorted(eventi):
        e = eventi[d]
        print(f'   {d:>5} ' + ' '.join(f'{(a_sim(e[n]) - t0):8.1f}' if n in e else f'{"-":>8}'
                                     for n, _ in EVENTI))
    rtf = np.diff(sim) / np.maximum(np.diff(parete), 1e-6)
    print(f'   RTF mediano dello stack {np.median(rtf[(rtf > 0) & (rtf < 2)]):.2f}')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for p in sys.argv[1:]:
        analizza(p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
