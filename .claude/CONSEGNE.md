# Passaggio di consegne

> **Aggiornato:** 2026-11-22 13:45 · **macchina:** jetson (`draupnir`, aarch64) · **commit di riferimento:** quello di queste consegne (`git log -1`)
> Lo riscrive Claude con `/consegne` a fine lavoro e lo legge con `/riprendi`. Resta sotto ~120 righe:
> i dettagli durevoli vanno in `.claude/memoria/` o in `LEGGIMI.md`, non qui.

## Obiettivo in corso
**Missione all'aperto sul drone vero**: "esplorare un'area di 30x30 mq, in un campo aperto" alla
velocita' raggiunta in simulazione (1 m/s) e "generare la mappa di esso" (mappa 3D colorata dalla
webcam). Pronto e provato in simulazione e col drone finto, **mai volato all'aperto con queste
modifiche**. Obiettivo sospeso dal 26 ott: sciame a 3 droni migliore del singolo.

## Dove eravamo rimasti
Sessioni 7, 8, 14 e 22 novembre (dettagli in memoria: `missione-aperto-recinto-virtuale.md`,
`collaudo-fase8-missione-webcam.md`, `verifiche-facoltative.md`, `batteria-stima-px4-drone-vero.md`).
- **7-8 nov**: fase 8 del collaudo = missione completa 5x5 m con mappa 3D dalla webcam, SUPERATA
  al chiuso sul drone vero l'8 nov (archivio `risultati_prova/2026-11-08_1044_...`, 52% colorata).
  Primo tentativo ALL'APERTO lo stesso giorno: atterrato dopo 1.8 s (FAST-LIO ~340 punti agganciati
  in volo contro la soglia di 500 del ponte della stima), foto bruciate dal sole.
- **14 nov**: recinto virtuale in `cloud_transform_bridge` (direzioni senza ritorni -> spazio libero,
  mai ostacoli), profili PX4 `scheda_sd_aperto`/`scheda_sd_chiuso`, `fast_lio_mid360_aperto.yaml`,
  `regola_webcam.py`, fasi 9-10 del collaudo. In simulazione (campo 50x50 senza recinto, 10
  ostacoli, area 30x30, 1 m/s): col recinto virtuale esplorazione conclusa (1147: 5009 m3, 578 foto,
  68% colorata; 1209 dal collaudo: fase 10 SUPERATA, 74% colorata), senza (1201) 201 frontiere
  aperte al tetto di 240 s.
- **22 nov** (solo codice e documenti, nessun volo):
  - l'utente ha messo `/etc/config.txt` col profilo all'aperto (MPC_XY_VEL_MAX 1.2, CRUISE 1.0)
    sulla SD del Pixhawk (righe della batteria: non si sa). Il lettore SD sulla porta USB-C del
    Jetson non funziona (porta in modo dispositivo): solo USB-A;
  - **giro a piedi (fase 9)**: prima tolto su richiesta dell'utente ("a cosa ti servirebbe?"), poi
    rimesso **facoltativo** ("lascialo, ma rendilo opzionale"): `FACOLTATIVE = {9}` in
    `collaudo.py`, INVIO nel menu lo salta, la fase 10 non lo richiede;
  - **fase 10, soglia di atterraggio della stima** (`SALUTE_MIN_AGGANCIATI`): `--soglia-aperto`, se
    no il giro a piedi di oggi (superato, < 12 h, stessa config di FAST-LIO: 60% del p5), se no 8 s a
    drone fermo sul decollo dopo la conferma finale (40% del p5, fra 150 e 500; 8 nov: da fermo
    620-680 -> 250, in volo 336). Giro di oggi NON superato -> chiede se volare lo stesso;
  - prova di `collaudo.py` col drone finto (script temporaneo, non nel repo): 25 controlli, 0 errori
    (giro superato e non, vecchio, altra config, soglia a mano, simulazione, fase 8 invariata; menu
    col registro vero `esiti_reale.json`, fasi 0-8 superate: INVIO propone la 10);
  - **trovati e NON corretti** i limiti fissi del sorvegliante (vedi Attenzione);
  - al campo l'utente usera' **un'altra webcam** (non la Logitech 046d:09c1);
  - spiegati all'utente la fase 10, cosa cambia all'aperto e come vengono evitati gli ostacoli
    (limiti in memoria: zona cieca bassa, fili, persone, superfici senza ritorni col recinto virtuale).

## Prossimi passi
1. **Limiti del sorvegliante**: chiedere all'utente se correggerli prima del volo all'aperto. Proposta
   gia' fatta: se la fase 10 passa `SALUTE_MIN_AGGANCIATI`, in volo 0.6 x soglia e nei controlli a
   terra soglia / 0.6; senza soglia (fase 8) 300 e 500 come ora. File:
   `ros2_ws/scripts/collaudo/sorveglia_missione.py` (righe 232 e 377) e il passaggio dei parametri
   in `run_reale_exploration.sh` (array `VOLO` ~riga 596 e i controlli a terra).
2. **Nuova webcam**, quando l'utente la collega al Jetson: dispositivo (`ls /dev/video*`), formati e
   fotogrammi/s a 640x480 (`v4l2-ctl` non e' installato: usare OpenCV), comandi di esposizione
   (`regola_webcam.py /dev/videoN --solo-leggi`). Al campo, nella fase 10, rimisurare posizione,
   inclinazione e campo di vista (in `misure_drone.json` ci sono quelli della Logitech:
   0.1,0,0.05 / 0 gradi / 50 gradi); se serve `CAMERA_DEV=/dev/videoN` e `CAMERA_HZ=<fps>`.
3. **Al campo** (area segnata, libera anche 2 m oltre, decollo al centro, nessuno nell'area):
   `cd ros2_ws/scripts/collaudo && FAST_LIO_CONFIG=fast_lio_mid360_aperto.yaml ./avvia_collaudo.sh reale /dev/ttyACM0 -- --area-aperto 15 --vel-aperto 0.5`,
   poi `9` se si vuole il giro a piedi (15x15), INVIO per la fase 10; dopo, 30x30 a 1 m/s senza le
   due opzioni. Durante gli 8 s di misura da fermo nessuno tocchi il drone.
4. Dopo il volo: dal `.ulg` sulla SD (pyulog) controllare MPC_XY_VEL_MAX 1.2, la batteria e i punti
   agganciati in volo rispetto a quelli da fermo o del giro (le frazioni 40% e 60% vengono da un solo
   volo, l'8 nov).
5. Dopo le prove all'aperto: `scheda_sd_chiuso/etc/config.txt` sulla SD (0.6 m/s al chiuso) e
   `regola_webcam.py /dev/videoN --automatica` (o scollegare la webcam).
6. Sciame (sospeso): campagna `risultati_prova/CAMPAGNA_SCIAME_2026-10-26_1523` incompleta; rifare
   office_small:3:1 ed empty_room:3:1 con la regola nuova di aiuto nella visita, poi parco,
   cubicle_office, open_field a 3 droni (V1 e V0); scrivere la sezione sciame in `LEGGIMI.md`.

## Attenzione
- **Sorvegliante con limiti fissi da interno** (NON corretto): in volo `sorveglia_missione.py` fa
  atterrare appena UN valore dei punti agganciati scende sotto 300 (riga 377), a terra ne pretende
  almeno 500 (riga 232), ignorando la soglia della fase 10. L'8 nov in volo all'aperto erano ~340:
  rischio concreto di atterraggio dopo pochi secondi. In simulazione non si vede (molti piu' punti).
- **Nessun radiocomando**: all'aperto le sole difese sono SPAZIO, sorvegliante e ponte della stima.
  Un radiocomando con kill switch collegato al Pixhawk resta fortemente consigliato.
- Mai provati sul drone vero: fasi 9 e 10, `fast_lio_mid360_aperto.yaml` (frequenza sul Jetson?),
  soglia da fermo, `regola_webcam.py`, recinto virtuale col Mid-360 vero, la nuova webcam.
- Il collaudo non legge i parametri di PX4 (il DDS non li porta): la fase 10 chiede conferma del
  profilo all'aperto. Il `config.txt` resta sulla SD e si riapplica a ogni avvio.
- A fine missione col recinto virtuale ~1 minuto fermo in volo sulle ultime frontiere prima di
  rientrare: conta nella batteria (stima della batteria sbagliata se mancano le righe BAT1_*).
- `tools/claude/inizio_lavoro.sh` segnala `exploration_manager` piu' nuovo della build: falso
  allarme, e' cambiato solo il launch (collegato con --symlink-install).
- Mai `git pull` ne' modifiche agli script con una run in corso (bash legge gli script mentre girano).
- Prove in simulazione isolate con `ROS_DOMAIN_ID=77` e `PF_MIN_RAM_GB=2` (RAM libera al limite con
  l'app desktop aperta).
- GPU: se Open3D/Gazebo vanno in crash, prima `journalctl -k -b 0 | grep -c "ACR bootstrap failed"`.

## Stato della macchina che ha scritto queste consegne
- Processi in esecuzione: nessuno (nessuna simulazione, nessuna missione).
- Nessuna ricompilazione necessaria: oggi solo script Python/bash e documenti; c2x_px4_bridge
  compilato il 14 nov; patch PX4 allineata.
- Solo su questa macchina (non in git): file pesanti di `risultati_prova/` (ply, png, pcd, ulg, foto
  delle mappe 3D) e gli `.ulg` del 25 ott e 7 nov in `collaudo/collaudo_registri/ulog_sd/`. Il mondo
  di prova si rigenera con
  `python3 scripts/make_open_field.py /tmp/campo_aperto.sdf --half 25 --senza-recinto --ostacoli 10`.
