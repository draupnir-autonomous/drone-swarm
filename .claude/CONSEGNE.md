# Passaggio di consegne

> **Aggiornato:** 2026-11-07 · **macchina:** jetson (`draupnir`, aarch64) · **commit di riferimento:** primo commit del repo
> Lo riscrive Claude con `/consegne` a fine lavoro e lo legge con `/riprendi`. Resta sotto ~120 righe:
> i dettagli durevoli vanno in `.claude/memoria/` o in `LEGGIMI.md`, non qui.

## Obiettivo in corso
Sciame a 3 droni più veloce, più coordinato e con mappa 3D finale migliore del drone singolo, su
tutti gli scenari (office_small, cubicle_office, parco, open_field, empty_room), con `VISITA=1` e
`VISITA=0`. Richiesta dell'utente: "vai avanti finché non stracci i risultati di un drone".

## Dove eravamo rimasti
Ultima sessione di lavoro tecnico: **26 ott 2026**, interrotta verso le 16:40 durante una campagna.
- Campagna `risultati_prova/CAMPAGNA_SCIAME_2026-10-26_1523` **incompleta**: fatte office_small
  3 droni V1 e V0, empty_room 3 droni V1; empty_room 3 droni V0 interrotta (run `1602_..._interrotta`).
  Nessuna `pagella.txt`.
- Risultati validi finora:
  - office_small: V1 sciame **86 s** vs singolo 125 s (foto 98% vs 91%); V0 sciame **57 s** vs 94 s;
  - cubicle_office: sciame ~320 s (run 1153) vs singolo 519 s (solo volo, senza ricostruzione mappa).
- Scambio mappe tra droni ora al 99-100% durante il volo (correzione del 26 ott, ore 15:20).
- Regola "aiuto nella visita" cambiata alle 15:58 (prendo la zona di un compagno solo se la mia
  distanza < 0.8 × il suo percorso): le run office_small:3:1 (1523) ed empty_room:3:1 (1544)
  usano la regola VECCHIA.
- 7 nov 2026: progetto reso portabile (git + kit Claude). Nessuna modifica al codice di volo.

## Prossimi passi
1. Rifare office_small:3:1 ed empty_room:3:1 con la regola nuova, ed empty_room:3:0.
2. Completare: parco, cubicle_office, open_field a 3 droni (V1 e V0) + singoli mancanti (parco,
   cubicle_office, open_field V0). Strumento: `ros2_ws/scripts/campagna_sciame.sh` con
   `RUN="mondo:n:visita ..."`, poi `pagella_sciame.py`.
3. Tabella finale con V1 e V0 per ogni mondo; scrivere in `LEGGIMI.md` la sezione sullo sciame
   (non c'è ancora: la bozza della sessione del 26 ott è andata persa con lo scratchpad).

## Attenzione
- Mai `git pull` né modifiche agli script con una run in corso: bash legge gli script mentre girano.
- Run lunghe staccate con `setsid nohup ...`, altrimenti muoiono con la sessione di Claude.
- Tempi in secondi SIMULATI (`TEMPO_SIM=1`): sulla Jetson lo sciame gira a RTF basso, il tempo reale
  è ~N volte più lungo. Su PC A/B (x86) i tempi reali saranno diversi, quelli simulati confrontabili.
- Su una macchina nuova la build completa richiede 20+ minuti (px4_msgs domina).

## Stato della macchina che ha scritto queste consegne
- Processi in esecuzione: nessuna simulazione attiva.
- Solo su questa macchina (non in git): file pesanti di `risultati_prova/` (ulg, ply, pcd, png, log).
