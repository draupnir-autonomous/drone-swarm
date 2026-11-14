---
name: missione-aperto-recinto-virtuale
description: "14 nov 2026: missione all'aperto 30x30 a 1 m/s con mappa 3D — recinto virtuale nella mappa (campo aperto senza ritorni), fasi 9 (giro a piedi) e 10 del collaudo, profili PX4, argparse coi valori negativi"
metadata:
  node_type: memory
  type: project
  originSessionId: d1720fc2-19ec-4719-8c66-3bf4a63e3faa
  modified: 2026-11-14T18:51:06.279Z
---

Richiesta (14 nov 2026): esplorare un campo aperto di 30x30 m alla velocita' della simulazione
(VEL_MAX 1.0) e farne la mappa 3D. L'utente ha scelto il **recinto virtuale** fra le opzioni; alla
domanda sul radiocomando con kill switch non ha risposto (si assume: nessuno).

Perche' serviva: un raggio del lidar senza ritorno non libera spazio (sdf_map raycasta solo i
punti), quindi in campo aperto il volume sopra il lidar resta ignoto e le frontiere non si
chiudono; per questo in simulazione open_field ha un recinto alto 5 m e il parco una siepe
(make_open_field.py, prepara_mondo.py "mondi aperti"). EnvSense (APERTO/CHIUSO) cambia solo la
spartizione fra droni, non lo spazio libero.

Recinto virtuale = parametro `recinto_virtuale` di cloud_transform_bridge (argomento di launch
`recinto_virtuale:=true`, `RECINTO_VIRTUALE=1` in run_reale_exploration.sh): celle di direzione di
2 gradi in terna MONDO; una cella senza ritorni da 5 scansioni (anche le 8 vicine) e dentro il campo
verticale del sensore (terna corpo: vero -7/+52, simulato +-50) riceve un punto virtuale a 60 m in
una direzione a caso della cella. sdf_map::inputPointCloud porta i punti fuori mappa o oltre
max_ray_length (20 m) sul bordo/troncati con tmp_flag 0 = libero, MAI ostacolo; li salta se il
punto troncato ha z < 0.2. I virtuali vanno solo in map_ros/cloud: cloud_mappa3d (mappa 3D) e il
filtro di sicurezza del ponte dei comandi (lidar/points) restano sui punti veri. ~3900 raggi
virtuali a scansione nel campo aperto simulato.

Prove in simulazione (14 nov, Jetson, run_reale_exploration.sh sim su campo 50x50 senza recinto
con 10 ostacoli, make_open_field.py --half 25 --senza-recinto --ostacoli 10, BOX -15,-15,15,15,
VEL_MAX 1.0, SIM_PX4_VEL_MAX 1.2, SIM_RECINTO=0): CON recinto virtuale (1139, 1147) esplorazione
conclusa (4-5 frontiere residue lasciate dopo 13 tentativi a vuoto = ~1 min fermo in volo),
5009 m3 esplorati, atterraggio a 8 cm, quota max 2.07-2.21 m (allarme era 2.3: ora QUOTA_ALLARME
2.6 nella fase 10), mappa 3D 2.45 M celle da 5 cm, 578 foto, 68% colorata; SENZA (1201, tetto
240 s): chiusa dal TEMPO MASSIMO con 201 frontiere aperte e 1996 m3. Prova completa dal collaudo
(`avvia_collaudo.sh sim <campo> -- --fase 10 --forza`, archivio 2026-11-14_1209_..._v1, registro
collaudo_registri/2026-11-14_120955_sim): fase 10 SUPERATA, esplorazione conclusa (1 frontiera
residua, "CODA"), 5001 m3, quota max 2.21 m (con l'allarme a 2.3 sarebbero stati 9 cm di margine),
uscita max dal recinto 0.08 m, mappa 3D 543 foto, 74% colorata. Da allora VEL_MAX passa come float
("1.0") cosi' l'archivio si chiama _v1.0 come le altre run.

Altri fatti non ovvi:
- argparse: un valore che comincia col meno ("-20,-20,20,20", "-0.05,0,0.1") e' preso per
  un'opzione ("expected one argument"): in run_reale_exploration.sh i vettori ora con "="
  (--ritaglio=, --lidar_offset=, --cam_pos=). Prima prova 2026-11-14_1139: registratore morto
  all'avvio, niente mappa 3D. Sul drone vero sarebbe successo con LIDAR_POS o CAMERA_POS negativi.
- PX4 simulato del collaudo e di run_reale sim ha il tetto 0.6 m/s (PX4_COLLAUDO): ora
  PX4_COLLAUDO_VEL_MAX/_CRUISE nell'airframe 22000 (patch salvata), SIM_PX4_VEL_MAX in run_reale.
- avvia_collaudo.sh esporta FAST_LIO_CONFIG col solo nome del file: collaudo.py lo passa alla
  missione col percorso completo, run_reale risolve i nomi in scripts/collaudo.
- Nei mondi generati, prepara_mondo aggiunge da solo un recinto ai mondi aperti: per provare il
  campo davvero aperto serve RECINTO=0 (SIM_RECINTO=0 in run_reale sim).
- run_reale_exploration.sh archiviava la mappa 3D rimasta in /tmp/c2x_run_reale dalla run
  precedente quando MAPPA_REALISTICA=0 (la 1201 aveva la mappa della 1147): ora la cartella si
  svuota a ogni avvio. Archivi reali e simulati fra ottobre e il 14 nov con MAPPA_REALISTICA=0 dopo
  una run con 1 possono avere lo stesso difetto.

Collaudo: fase 9 "FAST-LIO all'aperto" (a piedi, motori spenti: perimetro dell'area col drone in
mano; passa con area misurata entro il 15%, giro chiuso entro max(0.5 m, 0.5% del percorso), quota
20 cm, imbardata 5 gradi, >= 8 Hz, p5 agganciati >= 250; soglia = 60% del p5 fra 150 e 500 in
collaudo_registri/aperto.json, valida 12 h e con la stessa configurazione di FAST-LIO; registra
lidar/IMU/stima in risultati_prova/<data>_giro_a_piedi). Fase 10 "missione all'aperto": come la 8
(funzione missione() comune) con area = recinto, 1 m/s, 420 s, RECINTO_VIRTUALE, margine 2 m,
SALUTE_MIN_AGGANCIATI dalla fase 9, mappa 3D a 5 cm ritagliata (area + 5 m), regola_webcam.py
(esposizione manuale per il sole). Profili PX4 su scheda SD: scheda_sd_aperto (MPC_XY_VEL_MAX 1.2,
CRUISE 1.0) e scheda_sd_chiuso (0.6/0.5). fast_lio_mid360_aperto.yaml = point_filter_num 2,
det_range 60. MAI provati su hardware: fase 9, fast_lio_mid360_aperto.yaml, regola_webcam.py,
recinto virtuale col Mid-360 vero (nuvola non organizzata, scansione non ripetitiva).

**Why:** l'utente vuole la missione all'aperto con mappa 3D; il primo tentativo all'aperto (8 nov)
e' atterrato dopo 1.8 s per stima degradata, vedi [[collaudo-fase8-missione-webcam]].
**How to apply:** prima di volare all'aperto: profilo PX4, fase 9 superata nel campo, poi fase 10;
senza radiocomando area e velocita' ridotte le prime volte.
