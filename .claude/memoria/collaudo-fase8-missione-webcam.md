---
name: collaudo-fase8-missione-webcam
description: "7 nov 2026: fase 8 del collaudo = missione completa in 5x5 m con mappa 3D colorata dalla webcam; DAL_COLLAUDO, segnali, QoS e ritardo di cam2image, misure in misure_drone.json; superata l'8 nov 10:47 al chiuso (terzo tentativo)"
metadata:
  node_type: memory
  type: project
  originSessionId: d1720fc2-19ec-4719-8c66-3bf4a63e3faa
  modified: 2026-11-08T09:30:09.978Z
---

Dal 7 nov 2026 la fase 8 di `ros2_ws/scripts/collaudo/collaudo.py` non e' piu' "solo istruzioni"
(il vecchio esito con dettagli "solo istruzioni" viene scartato): recinto = spazio libero meno
0.5 m per lato (5x5 centrato -> BOX -2,-2,2,2), misure del drone chieste e salvate in
`collaudo_registri/misure_drone.json` (quota punto di riferimento, Mid-360, webcam pos/giu/hfov),
controllo webcam >= 3 fps, poi ferma SOLO la stima del collaudo e lancia
`run_reale_exploration.sh` con FOTO=1, DAL_COLLAUDO=1 (agente DDS e driver Livox restano quelli di
avvia_collaudo.sh: un agente riavviato puo' non ritrovare i topic di PX4), RIEPILOGO=<file>
letto per l'esito. Dopo la fase collaudo.py esce (stima ferma).

Fatti non ovvi trovati:
- avvia_collaudo.sh senza trap: alla caduta dell'SSH bash usciva SUBITO e fermava agente, lidar e
  stima col drone in volo. Ora trap (comando, non '') rimandato a fine collaudo.py.
- Un genitore Python non deve mettere SIG_IGN prima di lanciare uno script bash: i segnali ignorati
  all'avvio bash non li puo' piu' intercettare. Usare gestori vuoti (si azzerano all'exec).
- cam2image pubblica best effort: una sottoscrizione RELIABLE non riceve nulla. A 10 letture/s su
  una webcam a 30 fps legge fotogrammi accodati nel buffer V4L2 (~0.1 s vecchi): nella missione 30.
- La webcam sul Jetson il 7 nov e' una Logitech UVC 046d:09c1 (640x480 YUYV 30 fps), non la C170
  annotata il 19 ott.
- Con FAST-LIO odom_world = partenza + spostamento del LIDAR: telecamera in
  p + lidar_off + R (cam - lidar_off) (usato solo con --reale).
- Mid-360 -7 gradi: in una stanza 5x5 volando a 1 m il pavimento non entra in mappa.
- run_reale_exploration.sh era ROTTO dallo sciame: il via (/move_base_simple/goal) usciva con la
  QoS di serie (volatile) mentre l'FSM lo ascolta transient_local + reliable -> DDS lo scarta
  ("incompatible QoS ... DURABILITY"), l'FSM resta in WAIT_TRIGGER, il sorvegliante atterra dopo
  60 s. Corretto il 7 nov (--qos-durability transient_local --qos-reliability reliable, come
  run_multi). Lezione: cambiando la QoS di un topic dell'FSM, cercare TUTTI i publisher (anche
  negli script del drone vero, che non girano nelle campagne).
- Spazio piccolo: decollo e giro iniziale vedono gia' tutto il recinto, frontiere a zero prima del
  via; ExplorationReady vuole frontiere > 0, quindi niente via e il sorvegliante atterrava dopo
  60 s ("esplorazione non partita"; prova generale 2026-11-07_1714, BOX -2,-2,2,2 in
  cubicle_office). Corretto con `wait_ready.py --completa-dopo 5` (solo run_reale_exploration.sh):
  via anche con zero frontiere se volo_pronto, mappa che riceve map_ros/cloud (o frontiere viste
  prima) e zero frontiere da 5 s; l'FSM col via e zero frontiere va in PLAN_TRAJ -> NO_GRID ->
  IDLE e dopo min_explore_time_ (15 s) conclude "CONCLUSA", rientra e atterra.
- 8 nov, primo tentativo sul drone vero (registro 2026-11-08_101714_reale): missione NON partita,
  ferma ai controlli a terra ("nessuno pubblica /camera/image_raw"). cam2image lanciato da
  run_reale_exploration.sh con `-p frequency:=30` (CAMERA_HZ intero) abortisce:
  InvalidParameterTypeException, frequency e' double. Nel collaudo funzionava perche'
  collaudo_reale.launch.py passa 10.0. La prova generale in sim non lo poteva vedere (in sim la
  telecamera e' del modello, niente cam2image). `vivo camera` dopo 2 s non lo becca: sulla Jetson
  ros2 run impiega di piu' ad arrivare al crash. Corretto (FREQ_CAM convertito in float). Il resto
  dei controlli quel giorno era a posto: FAST-LIO 10 Hz, escursione 1.3 cm, scarto PX4 1 cm,
  batteria 16.99 V 100%, fasi 0-7 superate.
- 8 nov 10:25, secondo tentativo (registro 2026-11-08_102346_reale, archivio
  risultati_prova/2026-11-08_1024_reale_lio_v0.3), ALL'APERTO (erba, alberi): decollo ok, a 0.95 m,
  1.8 s dopo l'inizio del giro iniziale, lio_to_px4_bridge "STIMA FAST-LIO DEGRADATA" (5 scansioni
  sotto 500 agganciati, ultima 399, l1 86) -> land_request + flight_terminated -> LAND. NON e' stato
  PX4: l'FSM scrive "disarmato da PX4" per QUALSIASI flight_terminated (3 cause nel codice) e il
  sorvegliante "disarmato prima della fine del decollo, codice 2": la causa vera e' solo in
  rel_stima.log. Punti agganciati del Mid-360 vero: al chiuso il 7 nov (fasi 6-7) ~1500 da fermo e
  >=1218 in volo (l1 ~350); all'aperto 620-680 da fermo e 336 in volo (l1 ~170 / 71). La soglia 500
  e' tarata in sim (ufficio 1795, parco 1972) e il controllo a terra passava con 648 (margine 1.3x).
  Stima apparentemente coerente (z all'atterraggio 0.23 = partenza 0.22). Foto della webcam al sole
  saturate al 75-93% (mappa colorata al 5%): cam2image non regola l'esposizione.
- 8 nov 10:44, terzo tentativo AL CHIUSO (registro 2026-11-08_104413_reale, archivio
  risultati_prova/2026-11-08_1044_reale_lio_v0.3): fase 8 SUPERATA. Giro iniziale (raggio 0.5 m a
  1 m) che vede gia' tutto il recinto -> via a zero frontiere, rientro, atterraggio; 54 s in volo,
  quota max 1.01 m, fuori recinto 0.00, agganciati 1584-2071 in volo, 0 divergenze. Mappa 11.3 x
  12.6 x 3.1 m (stanza a L; pavimento quasi assente; soffitto grigio, webcam orizzontale), 52%
  colorata da 42 foto. Anteprime di ricostruisci_mappa.py TUTTE NERE (mesh 875k triangoli, nessun
  DISPLAY -> Filament a 1280x960 con 3 tentativi): rifatte a mano a 1280x720, mesh a 250k, fino
  a 40 tentativi (ne bastano 2-4). anteprime() corretta lo stesso giorno (1280x720, fino a 30
  tentativi, una vista rimasta nera non si salva e quella vecchia si cancella): provata solo con
  un renderer finto, la prova sulla GPU vera e' da fare (GPU guasta dopo il riavvio delle 11:13,
  vedi [[jetson-gpu-non-si-accende]]). Visualizzatore three.js in
  rel_mappa3d_1_1/mappa_3d.html (serve internet) e artifact privato
  https://claude.ai/artifact/16RuuwUMDrihUn3BGNHspu.
- Nei test sui processi mai filtrare per nome generico: sul Jetson root esegue
  /etc/systemd/nvmemwarning.sh con `sleep 300` (gruppo 939).

Verifiche 7 nov: prova generale `avvia_collaudo.sh sim -- --fase 8 --forza` (con ROS_DOMAIN_ID=77
per isolarla) superata alla 4a prova, archivio risultati_prova/2026-11-07_1748_reale-sim_...
(1714: niente via per frontiere a zero; 1722: via scartato per la QoS; la 3a non partita per RAM:
il preflight arrotonda per difetto e la Jetson con app desktop e altre sessioni ha ~3.1 GB liberi,
PF_MIN_RAM_GB=2 per la sola prova). collaudo.py usciva con core dump (rclpy.shutdown con ex.spin
attivo): corretto come in sorveglia_missione.py. Mai provati sul drone vero: DAL_COLLAUDO, webcam
in volo, misure reali.

Esiti: nel repo SWARM esiti_reale.json ha le fasi 0-7 superate (la 7 il 7 nov 18:23, quadrato
completato); la 8 non superata l'8 nov alle 10:17 e 10:25 (vedi sopra), SUPERATA alle 10:47 (al chiuso). In Drone_20261018_P la 7
e' superata (7 nov 10:14) ma la 6 no (7 nov 10:22: fuori recinto, quota max 3.10 m, poi batteria 14%
falsa). La fase 8 vuole 0-7 superate. Batteria: vedi [[batteria-stima-px4-drone-vero]] (config.txt
da applicare prima della missione, o il sorvegliante atterra per il 14% falso).

**Why:** l'utente vuole fare la missione completa col mappa 3D dal collaudo, sul drone vero.
**How to apply:** prima del volo: config.txt batteria applicato, fasi 0-7 superate nel repo da cui
si lancia, misure della webcam prese; colori spostati -> `ricostruisci_mappa.py <archivio>/rel_mappa3d_1_1
--cam_giu ... --cam_hfov ...`. Vedi [[drone-20261018-fast-livo2]].
