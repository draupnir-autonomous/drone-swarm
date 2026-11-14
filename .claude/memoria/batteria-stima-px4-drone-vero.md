---
name: batteria-stima-px4-drone-vero
description: "Carica batteria del drone vero sbagliata in volo (14% in volo, 69% a terra, 7 nov 2026) — PX4 la stima solo dalla tensione e il calo sotto carico resta quasi tutto scoperto; cause e rimedi proposti"
metadata:
  node_type: memory
  type: project
  originSessionId: 6b1ff80f-68f5-48b4-ab85-857509c5a0cc
  modified: 2026-11-07T15:24:28.345Z
---

7 nov 2026, collaudo fase 6: "SICUREZZA: batteria al 14%" 8 s dopo il decollo (in salita), a terra 69%.
Registro in `Drone_20261018_P/ros2_ws/scripts/collaudo/collaudo_registri/2026-11-07_101838_reale`:
i voli di oggi sono partiti dalla cartella VECCHIA 18_P, non dal repo SWARM. Stesso schema il
25 ott (25% fra 5 e 15 s dopo il decollo, 3 volte), allora attribuito tutto a BAT1_V_DIV.

Causa (codice PX4 v1.17 `src/lib/battery/battery.cpp`): con BAT1_CAPACITY = -1 `remaining` dipende
solo dalla tensione, lineare fra 3.60 e 4.05 V/cella (0.1 V = 22 punti). Il calo sotto carico viene
compensato con +R·I, dove R è stimata in volo (BAT1_R_INTERNAL = -1, si parte da 5 mΩ/cella) e solo
se la corrente è misurata. In v1.17 BAT_V_LOAD_DROP non esiste più. Da 69% a 14% mancano 0.25 V/cella
(1 V sul pacco) di compensazione. BAT1_CAPACITY non aiuta: la fusione gira a 100 Hz con peso 0.03 e
segue la tensione in ~1 s. Il commander, da armato, fa solo salire il livello di allarme
(`batteryCheck.cpp`:188): un calo sotto BAT_CRIT_THR 7% in salita fa atterrare PX4
(COM_LOW_BAT_ACT 2) fino al disarmo, qualunque cosa faccia lo script.
collaudo.py (`sicurezze`) e sorveglia_missione.py prendono `remaining` così com'è (DDS a 1 Hz) e
atterrano al primo campione sotto BATTERIA_MIN 0.15.

VERIFICATO sugli ulg (7 nov, copiati in `ros2_ws/scripts/collaudo/collaudo_registri/ulog_sd/`:
sess100 = 25 ott log100-107, sess101 = 7 nov log100-103; senza GPS PX4 scrive in sessNNN/logNNN.ulg):
- BAT1_V_DIV è ancora 18.1 e UXRCE_DDS_SYNCT ancora 1: il config.txt del 25 ott non è MAI stato
  applicato. Con pacco pieno PX4 legge 16.07-16.15 V → V_DIV giusto ≈ 18.8 (non 18.922).
- internal_resistance_estimate = 0 per tutto il 7 nov (il 25 ott 5.1): zero compensazione. Le
  variazioni rapide V/I a 5 Hz danno R negativa (tensione e corrente sfasate): probabile motivo.
- R effettiva dai salti decollo/atterraggio: mediana 32 mΩ pacco (8 mΩ/cella), oggi ~39; hover ~20 A.
- corrente plausibile (1-1.4 A a terra, 20 A in hover); oggi 1.42 Ah consumati in tutto.
- simulazione con V_DIV 18.8 + BAT1_R_INTERNAL 0.008: volo delle 10:23 83% prima, 81% in hover, 82% dopo
  (nel log: 49% → 14% → 24%).
- picchi 60-77 A al contatto col suolo in atterraggio (log102) → 12.7 V → CRITICAL PX4.

**Why:** la percentuale in volo fa atterrare il drone (script e PX4) con la batteria ancora a metà.
**How to apply:** impostare BAT1_V_DIV 18.8 e BAT1_R_INTERNAL 0.008 (config.txt sulla microSD, riavvio
vero), poi verificare V_DIV col tester; nello script persistenza e registro della batteria.
Strumenti di analisi: pyulog. Vedi [[drone-20261018-fast-livo2]].
