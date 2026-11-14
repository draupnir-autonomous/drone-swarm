# Passaggio di consegne

> **Aggiornato:** 2026-11-14 19:50 · **macchina:** jetson (`draupnir`, aarch64) · **commit di riferimento:** quello di queste consegne (`git log -1`)
> Lo riscrive Claude con `/consegne` a fine lavoro e lo legge con `/riprendi`. Resta sotto ~120 righe:
> i dettagli durevoli vanno in `.claude/memoria/` o in `LEGGIMI.md`, non qui.

## Obiettivo in corso
**Missione all'aperto sul drone vero**: "esplorare un'area di 30x30 mq, in un campo aperto" alla
velocita' raggiunta in simulazione (1 m/s) e "generare la mappa di esso" (mappa 3D colorata dalla
webcam). Tutto pronto e provato in simulazione, **mai volato all'aperto con queste modifiche**.
Obiettivo sospeso dal 26 ott: sciame a 3 droni migliore del singolo (vedi "Sciame" sotto).

## Dove eravamo rimasti
Sessioni 7, 8 e 14 novembre (dettagli in memoria: `collaudo-fase8-missione-webcam.md`,
`missione-aperto-recinto-virtuale.md`, `batteria-stima-px4-drone-vero.md`, `jetson-gpu-non-si-accende.md`).
- **7 nov, fase 8 del collaudo** = missione completa in 5x5 m con mappa 3D dalla webcam
  (`collaudo.py` -> `run_reale_exploration.sh` con `DAL_COLLAUDO=1`). Corretti: via dell'FSM mai
  ricevuto (QoS `transient_local` non seguita da run_reale dopo lo sciame), spazio piccolo senza via
  (`wait_ready.py --completa-dopo`), spegnimento dei componenti alla caduta dell'SSH in volo, core
  dump all'uscita di collaudo.py, registratore della mappa con la webcam vera (QoS, orologio).
- **8 nov, drone vero**: fase 8 SUPERATA al chiuso (10:47, archivio `risultati_prova/2026-11-08_1044_...`,
  mappa 52% colorata). Primo tentativo ALL'APERTO (10:25, `..._1024_...`): atterrato da solo dopo
  1.8 s, FAST-LIO agganciava ~340 punti in volo contro la soglia di 500 (al chiuso >= 1218); foto
  bruciate dal sole (75-93% sature). Dopo il riavvio delle 11:13 GPU spenta (ACR bootstrap failed);
  il 14 nov di nuovo funzionante.
- **14 nov, missione all'aperto** (scelta dell'utente: recinto virtuale; sul radiocomando nessuna
  risposta, quindi si assume che non ci sia):
  - **recinto virtuale** in `cloud_transform_bridge` (`recinto_virtuale:=true`, `RECINTO_VIRTUALE=1`):
    nelle direzioni senza ritorni del lidar punti virtuali a 60 m liberano lo spazio nella mappa
    del pianificatore (mai ostacoli); senza, in campo aperto le frontiere non si chiudono;
  - **fase 9** del collaudo: giro a piedi del perimetro, motori spenti; verifica FAST-LIO all'aperto
    e ricava la soglia di atterraggio (`collaudo_registri/aperto.json`), registra lidar/IMU;
  - **fase 10**: missione all'aperto 30x30 a 1 m/s (stessa funzione `missione()` della fase 8),
    margine del sorvegliante 2 m, quota d'allarme 2.6 m, mappa 3D a 5 cm ritagliata;
  - profili PX4 `collaudo/scheda_sd_aperto` (MPC_XY_VEL_MAX 1.2) e `scheda_sd_chiuso` (0.6),
    `fast_lio_mid360_aperto.yaml` (point_filter_num 2, det_range 60), `regola_webcam.py`;
  - prove in simulazione su un campo 50x50 senza recinto (`make_open_field.py --half 25
    --senza-recinto --ostacoli 10`, area 30x30, 1 m/s):

    | run | recinto virtuale | esito |
    |---|---|---|
    | `2026-11-14_1139` | si | esplorazione conclusa, atterrato; mappa 3D persa (argparse, corretto) |
    | `2026-11-14_1147` | si | conclusa, 5009 m3, atterrato a 8 cm, mappa 3D 578 foto, 68% colorata |
    | `2026-11-14_1201` | **no** | tetto di 240 s con **201 frontiere aperte**, 1996 m3 |
    | `2026-11-14_1209` | si, dal collaudo (fase 10) | **fase 10 SUPERATA**, 543 foto, 74% colorata |
  - corretti anche: vettori negativi passati ad argparse (registratore morto, mappa persa: ora
    `--opzione=valore`), archivio che copiava la mappa 3D della run precedente con
    `MAPPA_REALISTICA=0`, nome dell'archivio `_v1` invece di `_v1.0`.

## Prossimi passi
1. **Prima di andare al campo**: copia `ros2_ws/scripts/collaudo/scheda_sd_aperto/etc/config.txt`
   in `/etc/config.txt` della scheda SD del Pixhawk, riavvio VERO (stacca anche l'USB), poi togli
   il file. Se non e' gia' stato fatto, stessa cosa con `scheda_sd_una_volta/etc/config.txt`
   (batteria: BAT1_V_DIV 18.8, BAT1_R_INTERNAL 0.008). Batteria piena.
2. **Al campo** (area 30x30 segnata, libera anche 2 m oltre, decollo al centro):
   `cd ros2_ws/scripts/collaudo && FAST_LIO_CONFIG=fast_lio_mid360_aperto.yaml ./avvia_collaudo.sh reale /dev/ttyACM0`,
   poi **fase 9** (a piedi). Se non passa NON volare: la registrazione in
   `risultati_prova/<data>_giro_a_piedi/` serve a tarare FAST-LIO offline (strumento di riascolto
   ancora da scrivere).
3. **Fase 10** prima ridotta (`-- --area-aperto 15 --vel-aperto 0.5`), poi 30x30 a 1 m/s.
4. Verificare `regola_webcam.py` sulla webcam vera al sole (mai provato: controlli UVC di
   esposizione della Logitech 046d:09c1).
5. Dopo le prove all'aperto: `scheda_sd_chiuso` per tornare a 0.6 m/s al chiuso e
   `regola_webcam.py /dev/video0 --automatica`.
6. Sciame (sospeso): campagna `risultati_prova/CAMPAGNA_SCIAME_2026-10-26_1523` incompleta;
   rifare office_small:3:1 ed empty_room:3:1 con la regola nuova di aiuto nella visita, poi parco,
   cubicle_office, open_field a 3 droni (V1 e V0); scrivere la sezione sciame in `LEGGIMI.md`.

## Attenzione
- **Nessun radiocomando**: all'aperto le sole difese sono SPAZIO, sorvegliante e ponte della
  stima. Un radiocomando con kill switch collegato al Pixhawk resta fortemente consigliato.
- Mai provati sul drone vero: fase 9, `fast_lio_mid360_aperto.yaml` (frequenza sul Jetson?),
  `regola_webcam.py`, recinto virtuale col Mid-360 vero (nuvola non organizzata).
- Il collaudo non legge i parametri di PX4 (il DDS non li porta): la fase 10 chiede conferma del
  profilo all'aperto; con 0.6 m/s il drone vola piu' lento del piano.
- A fine missione col recinto virtuale ~1 minuto fermo in volo sulle ultime frontiere (13
  tentativi a vuoto dell'FSM) prima di rientrare: conta nella batteria.
- `tools/claude/inizio_lavoro.sh` segnala `exploration_manager` piu' nuovo della build: falso
  allarme, e' cambiato solo il launch (collegato con --symlink-install); l'eseguibile e' aggiornato.
- Mai `git pull` ne' modifiche agli script con una run in corso (bash legge gli script mentre girano).
- Prove in simulazione isolate con `ROS_DOMAIN_ID=77` (PX4 SITL lo segue) e `PF_MIN_RAM_GB=2`:
  sulla Jetson con l'app desktop aperta la RAM libera e' sul limite dei 3 GB.
- GPU: se Open3D/Gazebo vanno in crash, prima `journalctl -k -b 0 | grep -c "ACR bootstrap failed"`.

## Stato della macchina che ha scritto queste consegne
- Processi in esecuzione: nessuno (nessuna simulazione, nessuna missione).
- c2x_px4_bridge ricompilato il 14 nov (recinto virtuale); patch PX4 salvata (airframe 22000:
  `PX4_COLLAUDO_VEL_MAX`/`_CRUISE`), copia di build gia' allineata.
- Solo su questa macchina (non in git): file pesanti di `risultati_prova/` (ply, png, pcd, ulg,
  foto delle mappe 3D) e gli `.ulg` del 25 ott e 7 nov in `collaudo/collaudo_registri/ulog_sd/`.
  Il mondo di prova `campo_aperto.sdf` stava nella cartella temporanea: si rigenera col comando sopra.
