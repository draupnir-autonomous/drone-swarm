---
name: missione-aperto-recinto-virtuale
description: "14 nov 2026: missione all'aperto 30x30 a 1 m/s con mappa 3D — recinto virtuale nella mappa (campo aperto senza ritorni), fasi 9 (giro a piedi, facoltativa dal 22 nov) e 10 del collaudo, soglia della stima dal giro o da fermo, limiti fissi del sorvegliante da correggere, altra webcam al campo, ostacoli, profili PX4, argparse coi valori negativi"
metadata:
  node_type: memory
  type: project
  originSessionId: d1720fc2-19ec-4719-8c66-3bf4a63e3faa
  modified: 2026-11-22T11:39:46.822Z
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

Collaudo: fase 9 "FAST-LIO all'aperto", FACOLTATIVA dal 22 nov (a piedi, motori spenti: perimetro
dell'area col drone in mano; passa con area misurata entro il 15%, giro chiuso entro max(0.5 m,
0.5% del percorso), quota 20 cm, imbardata 5 gradi, >= 8 Hz, p5 agganciati >= 250; soglia = 60%
del p5 fra 150 e 500 in collaudo_registri/aperto.json, valida 12 h e con la stessa configurazione
di FAST-LIO; registra lidar/IMU/stima in risultati_prova/<data>_giro_a_piedi). Il 22 nov l'utente
l'ha fatta prima togliere ("a cosa ti servirebbe?"), poi rimettere "opzionale, non obbligatoria":
FACOLTATIVE = {9} in collaudo.py (fasi_mancanti non la pretende, prossima_fase/INVIO la salta).
Fase 10 "missione all'aperto": come la 8 (funzione missione() comune) con area = recinto, 1 m/s,
420 s, RECINTO_VIRTUALE, margine 2 m, QUOTA_ALLARME 2.6, mappa 3D a 5 cm ritagliata (area + 5 m),
regola_webcam.py (esposizione manuale per il sole). SALUTE_MIN_AGGANCIATI: --soglia-aperto, se no
il giro a piedi valido, se no 40% del p5 misurato 8 s a drone fermo sul decollo dopo la conferma
finale, fra 150 e 500 (8 nov: da fermo 620-680, in volo 336 = 52%: 250 resta sotto il volo
normale; sotto 375 da fermo chiede se volare). Giro a piedi di oggi NON superato: la fase 10 chiede
se volare lo stesso (poi soglia da fermo). Profili PX4 su scheda SD: scheda_sd_aperto (MPC_XY_VEL_MAX 1.2,
CRUISE 1.0) e scheda_sd_chiuso (0.6/0.5). fast_lio_mid360_aperto.yaml = point_filter_num 2,
det_range 60. MAI provati su hardware: fasi 9 e 10, fast_lio_mid360_aperto.yaml, regola_webcam.py,
recinto virtuale col Mid-360 vero (nuvola non organizzata, scansione non ripetitiva).

22 nov: l'utente ha messo da se' /etc/config.txt col profilo all'aperto sulla SD del Pixhawk (non
verificato: dal Jetson i parametri non si leggono, il DDS non li porta; controllo nel primo .ulg).
Il lettore SD sulla porta USB-C della Jetson non viene enumerato (porta in modo dispositivo, quella
del gadget L4T-README): solo porte USB-A. Per cambiare parametri senza scheda SD servirebbe MAVLink
su TELEM1 (gia' attivo a 57600, MAV_0_CONFIG 101) con un adattatore USB-seriale 3.3 V.

22 nov, NON corretto: sorveglia_missione.py ha limiti fissi tarati al chiuso che ignorano
SALUTE_MIN_AGGANCIATI: in volo atterra se UN valore dei punti agganciati e' < 300 (riga 377,
istantaneo), nei controlli a terra pretende >= 500 (riga 232). All'aperto (8 nov ~340 in volo)
rischia di far atterrare subito; in simulazione non si vede (molti piu' punti). Proposta: in volo
0.6 x soglia (300 su 500 come ora), a terra soglia / 0.6, invariato se SALUTE_MIN_AGGANCIATI
non e' dato (fase 8).

22 nov: al campo si usera' un'altra webcam (non la Logitech 046d:09c1, 640x480 YUYV 30 fps, delle
prove al chiuso). Dipende dalla webcam: le misure in collaudo_registri/misure_drone.json
(camera_pos, camera_giu, camera_hfov: quelle salvate sono della Logitech, HFOV 50), CAMERA_DEV
(solo /dev/videoN), CAMERA_HZ (di serie 30: va messa uguale ai fotogrammi/s della webcam a
640x480, se no cam2image legge fotogrammi vecchi fino a 0.1 s), CAMERA_RITARDO 0.05 (non si
corregge dopo il volo). registra_mappa usa un obiettivo senza distorsione (fx dalla larghezza
dell'immagine e dall'HFOV): con un grandangolo i colori ai bordi sono meno precisi. La webcam deve
guardare in avanti e stare dritta (si misura solo l'inclinazione in giu'). regola_webcam.py e'
generico (OpenCV V4L2: AUTO_EXPOSURE 1 = manuale, 3 = automatica; EXPOSURE in 100 us).
Posizione, inclinazione e HFOV si correggono dopo il volo con ricostruisci_mappa.py.

Ostacoli nel recinto (spiegato all'utente il 22 nov): mappa a 10 cm gonfiata di 0.3 m,
sdf_map.optimistic e search.optimistic false (lo spazio ignoto non si attraversa), dist0 0.7 m,
ripianificazione ogni 0.2 s, filtro di sicurezza del ponte dei comandi sui punti veri (da 0.9 m
riduce la componente verso l'ostacolo, a 0.5 m la annulla) e limite di frenata (0.45 m dal centro),
riferimento fra 1.0 e 2.0 m di quota (mai sotto 0.8). Effetto collaterale del recinto virtuale: una
direzione SENZA ritorni diventa spazio libero invece che ignoto, quindi un ostacolo che non rimanda
il raggio (vetro, acqua, fili) per il pianificatore e' libero. Zona cieca del Mid-360 (-7 gradi): a
1 m di quota un ostacolo di 50 cm resta in vista solo oltre ~5 m, poi resta nella mappa; il rischio
e' cio' che sta vicino al decollo. Persone in movimento: la mappa non e' fatta per loro.

**Why:** l'utente vuole la missione all'aperto con mappa 3D; il primo tentativo all'aperto (8 nov)
e' atterrato dopo 1.8 s per stima degradata, vedi [[collaudo-fase8-missione-webcam]].
**How to apply:** prima di volare all'aperto: profilo PX4 sulla SD, se si vuole la fase 9 nel campo,
poi la fase 10 (la prima volta ridotta);
senza radiocomando area e velocita' ridotte le prime volte.
