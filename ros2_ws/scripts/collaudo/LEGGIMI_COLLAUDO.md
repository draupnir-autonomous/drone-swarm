# Collaudo del drone vero, passo per passo

Strumenti in questa cartella (ottobre 2026):

| file | cosa fa |
|---|---|
| `PROCEDURA_PROVE_REALI.md` | **la lista di controllo in ordine** (casa, posto, collaudo, missione, dopo il volo), da tenere sul campo |
| `collaudo.py` | il programma del collaudo: 9 fasi in ordine, controlli automatici, levette virtuali da tastiera o joystick, arresto d'emergenza, registro; la fase 8 e' la missione completa con la mappa 3D |
| `avvia_collaudo.sh` | avvia tutto (agente DDS, driver Livox, FAST-LIO, ponte verso PX4) e apre `collaudo.py`; con `sim` lo stesso contro la simulazione |
| `collaudo_reale.launch.py` | FAST-LIO sul Mid-360 vero e il ponte della stima verso PX4 (tempo reale, niente Gazebo) |
| `fast_lio_mid360.yaml` | configurazione di FAST-LIO per il Mid-360 vero |
| `collaudo_px4.params` | parametri PX4 da caricare con QGroundControl prima del primo volo |
| `sorveglia_missione.py`, `prepara_reale.py` | per la missione di esplorazione sul drone vero (`../run_reale_exploration.sh`, dopo il collaudo): controlli a terra, sorveglianza con emergenza, configurazione del recinto |
| `collaudo_registri/` | un registro per sessione e quali fasi sono superate, separati per simulazione e drone vero (`esiti_sim.json`, `esiti_reale.json`) |

## Prima di tutto: la sicurezza (leggere, non saltare)

Il drone **non ha un radiocomando**. Quindi non c'e' un pilota che possa riprendere il
controllo con le mani, e l'arresto dei motori e' solo software: funziona finche' il
programma gira e il collegamento regge. Per questo:

- **Le eliche si montano solo dalla fase 5.** Fasi 0-4 sempre senza eliche.
- **Fase 5 con il drone legato** (corda corta, 30-50 cm di gioco, fissata a un peso o al
  pavimento) o dentro una rete. Al chiuso, sempre: protezioni delle eliche montate.
- **Nessuno entro 3 m** dal drone in volo; l'operatore dietro un riparo o a distanza.
- **SPAZIO = emergenza**: spegne i motori subito, il drone cade. Usalo se il drone va dove
  non deve: da 1 m una caduta costa meno di un volo fuori controllo.
- Prima di ogni volo: batteria carica (> 50%), viti delle eliche, cavi fissati, lidar pulito.
- **Il luogo deve avere struttura vicina**: pareti, colonne, scaffali o scatoloni entro
  ~10 m in tutte le direzioni. In simulazione, in una stanza quasi vuota (`empty_room`:
  pavimento e soffitto grandi, quattro pareti corte a 6 m), FAST-LIO **non ha visto** uno
  spostamento vero di 4.8 m: la stima diceva fermo, PX4 non correggeva e il drone
  scivolava, mentre i numeri di salute (punti agganciati, autovalore minimo) sembravano
  buoni. La fase 3 (drone spostato a mano di 50 cm) va fatta **nel punto da cui si
  decolla**: e' il controllo che scopre questo caso.
- **Guarda il drone, non lo schermo.** Se si sposta da solo mentre lo schermo dice fermo,
  la stima non vede il movimento: `l` se e' lento e lontano dagli ostacoli, altrimenti SPAZIO.
- Il collaudo e' fatto per l'**interno di un capannone o palestra vuoti**. Le regole per il
  volo (in Italia ENAC/EASA: registrazione operatore, assicurazione, categoria Open o
  autorizzazioni) restano a carico di chi vola, anche al chiuso se lo spazio non e' privato.
- Consigliato davvero: un radiocomando con interruttore di spegnimento (kill switch)
  collegato al Pixhawk anche solo come sicurezza. Con il radiocomando acceso PX4 da' la
  precedenza alle levette vere (`COM_RC_IN_MODE` = 2 invece di 1).

Come il programma protegge il volo:
- se `collaudo.py` si ferma, il Jetson si blocca o il cavo USB si stacca, PX4 non riceve
  piu' levette e dopo 0.5 s **atterra da solo** (`NAV_RCL_ACT` = Land);
- se cade la sessione SSH (il programma riceve SIGHUP) comanda l'atterraggio;
- in volo atterra da solo se: la stima di FAST-LIO e' ferma da 0.5 s, i punti agganciati
  scendono sotto 300, la posizione di PX4 non e' valida, il drone esce dal recinto (raggio
  dal punto di decollo o quota massima), la batteria scende sotto il 15%;
- `lio_to_px4_bridge` chiede l'atterraggio se FAST-LIO aggancia meno di 500 punti per 5
  scansioni o se la stima diverge (velocita' assurde).

## Preparazione (una volta)

1. **Parametri PX4**: con QGroundControl e il Pixhawk collegato al portatile via USB,
   salva i parametri attuali (Parameters > Tools > Save to file), poi carica
   `collaudo_px4.params` (Load from file) e riavvia. `EKF2_EV_POS_X/Y/Z` restano a zero
   (vedi il commento nel file); **misura invece** quota da terra del punto di riferimento del
   drone appoggiato e posizione del Mid-360 rispetto a quel punto: servono alla missione
   (`PARTENZA_Z`, `LIDAR_POS` di `run_reale_exploration.sh`).
   In QGroundControl controlla anche: airframe giusto, sensori calibrati (accelerometro,
   livello orizzontale), motori in ordine e verso giusto (Actuators). Metti `MPC_THR_HOVER`
   vicino alla spinta di hover vera (se troppo bassa il drone resta a terra mentre PX4 lo
   crede in volo e poi si stacca di colpo: visto in simulazione; `collaudo.py` spegne i
   motori se 8 s dopo il decollo il drone e' ancora sotto 10 cm).
2. **DDS sul cavo USB** (PX4 di serie mette MAVLink sull'USB). Sulla scheda SD del
   Pixhawk crea `/etc/extras.txt` con:
   ```
   uxrce_dds_client stop
   uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
   ```
   e imposta in QGroundControl `SYS_USB_AUTO` = 0, per ULTIMO (non e' nel file dei parametri). Da quel momento
   QGroundControl non si collega piu' via USB: per tornare indietro togli `extras.txt` e
   rimetti `SYS_USB_AUTO` = 2. **Da verificare alla fase 0**: PX4 non prevede ufficialmente
   il DDS sull'USB; se non funziona, l'alternativa standard e' un cavo seriale dalla porta
   TELEM2 a un adattatore USB-seriale sul Jetson (`UXRCE_DDS_CFG` = 102, `SER_TEL2_BAUD`
   921600, e `./avvia_collaudo.sh reale /dev/ttyUSB0`).
3. **Driver del lidar**: installato il 19 ottobre 2026. Livox-SDK2 (commit c0796f0, MIT)
   e' compilato in `terze_parti/Livox-SDK2` e installato DENTRO il progetto in
   `terze_parti/livox_sdk2` (senza sudo); il driver completo `livox_ros_driver2` (commit
   2144554, MIT) e' in `ros2_ws/src/terze_parti/livox_ros_driver2` e collega la libreria
   statica dell'SDK, quindi funziona anche dopo aver spostato il progetto. Ha preso il posto
   del pacchetto con i soli messaggi (copia in `terze_parti/livox_ros_driver2_solo_messaggi`):
   stessi messaggi, FAST-LIO e FAST-LIVO2 ricompilati sopra. Si avvia con
   `ros2 launch livox_ros_driver2 msg_MID360_launch.py` (lo fa `avvia_collaudo.sh reale`):
   nuvola `livox_ros_driver2/CustomMsg` su `/livox/lidar`, IMU su `/livox/imu`.
   **Non usare il `build.sh` del driver**: cancella `build/` e `install/` dell'intero
   spazio di lavoro (per questo nella copia non c'e').
   Configurazione: `ros2_ws/src/terze_parti/livox_ros_driver2/config/MID360s_config.json` (il lidar
   del drone e' un **Mid-360S**, tipo 35: `MID360_config.json` vale solo per il Mid-360, tipo 9,
   e con quello il driver trova il lidar ma non lo configura).
   Di serie il lidar e' a `192.168.1.12` e il computer a `192.168.1.5`; il Mid-360 vero e' di
   solito a `192.168.1.1XX` con XX = ultime due cifre del numero di serie (etichetta).
4. **Rete del lidar**: il Mid-360 esce di fabbrica su 192.168.1.1xx con il computer a
   192.168.1.50. Il Wi-Fi di questo Jetson e' su 192.168.1.x: **le due reti si
   sovrappongono** e i pacchetti del lidar possono finire sul Wi-Fi. Soluzione: cambia
   l'indirizzo del lidar (Livox Viewer 2) e della porta Ethernet del Jetson su un'altra
   rete, per esempio 192.168.10.x, e aggiorna `MID360_config.json`.
5. **Montaggio del Mid-360**: il ponte verso PX4 assume il lidar con gli assi del drone
   (x avanti, y sinistra, z su), cioe' montato dritto col davanti verso il muso. La fase 3
   lo verifica; se e' montato diversamente, prima di volare va aggiunta la rotazione nel
   ponte.
6. **SSH dal portatile al Jetson** con il controllo del collegamento, cosi' una caduta del
   Wi-Fi chiude la sessione in pochi secondi e il programma fa atterrare il drone:
   ```
   ssh -o ServerAliveInterval=1 -o ServerAliveCountMax=3 utente@jetson
   ```
   e sul Jetson, in `/etc/ssh/sshd_config`: `ClientAliveInterval 1`, `ClientAliveCountMax 3`.
   Meglio ancora: il portatile collegato al Jetson con un cavo Ethernet o con un router
   vicino, non sulla rete di casa.

## Ogni giorno di prove

```bash
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh reale /dev/ttyACM0
```
Con un gamepad collegato al Jetson:
```bash
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh reale /dev/ttyACM0 -- --joystick /dev/input/js0
```
Prima, a casa, la stessa sequenza in simulazione (stesso programma, PX4 simulato con gli
stessi limiti di `collaudo_px4.params`, mondo `cubicle_office`; `GUI=1` per vedere Gazebo):
```bash
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh sim
```

Il programma mostra le fasi con `[ OK]` / `[ NO]`; INVIO esegue la prossima non superata.
Non lascia saltare una fase non superata (si puo' con `--forza`, ma resta nel registro), tranne
la 9, facoltativa: la 10 non la richiede e INVIO nel menu la salta.

| fase | eliche | cosa si fa | superata se |
|---|---|---|---|
| 0 collegamento | no | il programma legge lo stato di PX4 e prova le levette virtuali | PX4 risponde, accetta le levette, parametri caricati, batteria > 50% |
| 1 sensori | no | misura 5 s | lidar ~10 Hz, IMU >= 100 Hz con gravita' giusta |
| 2 FAST-LIO fermo | no | drone a terra, nessuno lo tocca, 30 s | stima > 8 Hz, si muove meno di 5 cm, > 500 punti agganciati |
| 3 assi e stima | no | inclini il drone a mano avanti e a destra, lo sposti avanti di 50 cm, lo ruoti a sinistra di 90 gradi | PX4 e FAST-LIO concordano su verso e misura di ogni movimento; EKF2 usa la visione |
| 4 motori | **no** | ogni motore gira 1.5 s al 15% (controlli posizione e verso), poi armamento e prova dell'emergenza | motori giusti, emergenza in meno di 0.5 s |
| 5 hover trattenuto | si, **drone legato** | `t` decolla a 0.5 m, 30 s fermo, `l` atterra | nessun allarme, atterrato e disarmato |
| 6 volo manuale | si | `t` a 1 m, piccoli spostamenti con le frecce (~0.16 m/s), recinto 2 m | nessun allarme |
| 7 quadrato automatico | si | `t`, poi `o`: quadrato di 1 m a 0.3 m/s in offboard, poi fermo; `l` atterra | nessun allarme |
| 8 missione completa | si | spazio libero 5 x 5 m, drone al centro, webcam collegata: misure del drone, poi la missione di esplorazione (scrivi `VOLA`) e la mappa 3D colorata | atterrato da solo sul decollo, esplorazione conclusa, mappa 3D con le foto della webcam |
| 9 FAST-LIO all'aperto (**facoltativa**) | **no, motori spenti** | all'aperto, drone in mano a ~1 m: dal centro dell'area al bordo, tutto il perimetro, ritorno al centro | lato dell'area misurato entro il 15%, giro chiuso entro 0.5 m, FAST-LIO >= 8 Hz, abbastanza punti agganciati; ricava la soglia per la fase 10 |
| 10 missione all'aperto | si | area 30 x 30 m (drone al centro) a 1 m/s, recinto virtuale, profilo PX4 all'aperto; soglia della stima dalla fase 9 o, senza, misurata a drone fermo | come la fase 8 |

Tasti in volo: frecce = sposta, `w`/`s` = su/giu', `a`/`d` = ruota, `t` = decolla (in
offboard, con la quota che sale a 0.4 m/s fino a quella della fase, poi resta fermo in
POSCTL; una levetta o `h` interrompe la salita e passa al comando manuale),
`h` = fermo (interrompe anche l'offboard), `l` = atterra, **SPAZIO = emergenza**, `q` = esci
(solo a terra). Nell'offboard della fase 7 una qualsiasi freccia riprende il comando manuale.

Dopo ogni volo: il registro della sessione e' in `collaudo_registri/<data_ora>/`, il
registro di volo di PX4 (`.ulg`) sulla scheda SD. Con il `.ulg` si misura il ritardo vero
della stima (per `EKF2_EV_DELAY`) con lo stesso metodo di `scripts/confronta_stima.py`.

## Fase 8: la missione completa con la mappa 3D (novembre 2026)

Con le fasi 0-7 superate, la fase 8 fa volare la missione di esplorazione vera
(`../run_reale_exploration.sh`) in uno **spazio libero quadrato di 5 x 5 m** col drone al
centro, e alla fine costruisce la **mappa 3D colorata con le foto della webcam**. Fino al 7
novembre 2026 la fase 8 dava solo istruzioni: quel vecchio esito non conta piu'.

Prima di cominciare:
- spazio libero 5 x 5 m (pareti o segni a terra ai bordi), soffitto ad almeno **2.8 m**, niente
  sotto 1 m vicino al drone (il Mid-360 non vede in basso vicino: -7 gradi); struttura vicina
  per FAST-LIO (in una stanza di 5 m le pareti bastano);
- webcam collegata (`/dev/video0`, o `CAMERA_DEV=/dev/videoN ./avvia_collaudo.sh ...`): la fase
  controlla che pubblichi;
- batteria carica: il sorvegliante fa atterrare sotto il 15% **come lo legge PX4** (vedi la
  stima sbagliata in volo nelle note del 7 novembre 2026: prima della missione la
  `scheda_sd_una_volta/etc/config.txt` con `BAT1_V_DIV` e `BAT1_R_INTERNAL` va applicata).

Cosa succede (`./avvia_collaudo.sh reale /dev/ttyACM0`, poi `8`):
1. **Recinto**: drone al centro dello spazio col muso verso un lato (se no, le distanze dai
   quattro bordi). Il recinto della missione e' lo spazio meno 0.5 m per lato, per 5 x 5 m
   `BOX="-2,-2,2,2"`: il sorvegliante fa atterrare 0.5 m oltre, cioe' al bordo dello spazio; le
   pareti appena fuori dal recinto restano nella mappa del pianificatore, che ne sta lontano.
   Altri spazi: `-- --area 6` (6 x 6 m) o `-- --box xmin,ymin,xmax,ymax`.
2. **Misure del drone**, da un punto di riferimento a scelta (per esempio il centro del
   Pixhawk), assi x avanti, y sinistra, z su: quota da terra del punto col drone appoggiato,
   posizione del Mid-360, posizione dell'obiettivo della webcam, sua inclinazione verso il basso
   e campo di vista orizzontale (a distanza D da una parete si segnano i bordi sinistro e destro
   dell'immagine, distanti L: campo = 2*atan(L/(2*D))). INVIO tiene il valore proposto; restano
   in `collaudo_registri/misure_drone.json` (versionato: valgono per il drone su ogni computer).
3. **Webcam**: deve dare almeno 3 fotogrammi/s.
4. Conferma, poi il collaudo **ferma la sua stima** (FAST-LIO, ponte, webcam: FAST-LIO ha
   l'origine dove e' partito, e dopo la fase 3 e i voli non e' piu' il punto di decollo) e passa
   la mano a `run_reale_exploration.sh` con `DAL_COLLAUDO=1`: **agente DDS e driver del lidar
   restano quelli del collaudo** (un agente riavviato a volte non ritrova i topic di PX4).
5. Lo script della missione fa i suoi controlli a terra (anche la webcam), chiede `VOLA`, e il
   drone decolla subito a 1 m, gira su se stesso, esplora a 0.3 m/s per al massimo 240 s
   (`-- --vel-missione`, `-- --tempo-missione`), rientra e atterra sul decollo. In uno spazio
   piccolo il giro iniziale puo' vedere gia' tutto il recinto: allora l'esplorazione si conclude
   subito ("CONCLUSA", dopo ~15 s di attesa) e il drone atterra sul posto, che e' il decollo
   (prima il via non partiva e il sorvegliante lo faceva atterrare dopo un minuto come per un
   guasto: vedi `wait_ready.py --completa-dopo`). In volo resta il sorvegliante: **SPAZIO =
   emergenza**, `l` = atterra, atterraggio da solo fuori dal recinto, con stima ferma o
   peggiorata, batteria bassa, tempo scaduto o SSH caduta.
6. A terra: mappa 3D (superficie dal lidar a 2 cm, colori dalle foto della webcam, qualche
   minuto), archivio in `risultati_prova/<data>_reale_lio_v0.3/` con `rel_mappa3d_1_1/`
   (`mappa_realistica.ply`, `nuvola_colorata.ply`, anteprime PNG, le foto con la posa di ogni
   scatto e i punti lidar). La fase e' superata se il drone e' atterrato da solo, l'esplorazione
   e' conclusa (anche per tetto di tempo) e la mappa ha i colori delle foto.
7. Il collaudo si chiude (la sua stima non c'e' piu'): per altre fasi si rilancia
   `avvia_collaudo.sh`, che alla chiusura ferma anche agente e driver.

**Colori spostati rispetto alle forme** (misure della webcam approssimate): si correggono dopo,
senza rivolare, perche' l'archivio tiene foto, posa del drone a ogni scatto e punti lidar:
```bash
python3 ros2_ws/scripts/ricostruisci_mappa.py risultati_prova/<run>/rel_mappa3d_1_1 --cam_giu 12 --cam_hfov 48
```
(anche `--cam_pos x,y,z`); poi le misure giuste vanno ridate alla fase 8 successiva.

**Cosa aspettarsi dalla mappa**: il Mid-360 vede da -7 a +52 gradi, quindi volando a 1 m il
pavimento entra solo oltre ~8 m: in uno spazio di 5 m la mappa ha pareti, soffitto e oggetti
alti, quasi niente pavimento. La webcam colora solo cio' che il lidar ha misurato.

Prova generale a casa: `./avvia_collaudo.sh sim`, poi `8` (con le fasi 0-7 della simulazione
superate, o `-- --fase 8 --forza`): stessa sequenza con la telecamera simulata, senza domande
sulle misure.

Se la sessione SSH cade o arriva Ctrl+C, `avvia_collaudo.sh` spegne agente, lidar e stima solo
**dopo** che `collaudo.py` (e la missione) hanno finito, cioe' a drone atterrato: fino al 7
novembre 2026 alla caduta dell'SSH li spegneva subito, anche col drone in volo.

## All'aperto: fasi 9 e 10 (novembre 2026)

Obiettivo: esplorare un'area di **30 x 30 m in un campo aperto a 1 m/s** e farne la mappa 3D.
Tre cose cambiano rispetto al chiuso, e la prima prova all'aperto (8 novembre 2026, fase 8 su un
prato con alberi) le ha mostrate tutte:
- **FAST-LIO aggancia meno punti**: 620-680 da fermo e ~340 in volo contro ~1500 al chiuso; con
  la soglia di 500 (tarata al chiuso e in simulazione) `lio_to_px4_bridge` ha fatto atterrare il
  drone 1.8 s dopo il decollo. Non e' un difetto della soglia: all'aperto la stima e' davvero piu'
  debole, e senza pareti che fermino il drone una stima sbagliata e' un drone che se ne va. La
  soglia si ricava nel campo: dal giro a piedi (fase 9, facoltativa) o, senza, a drone fermo sul
  decollo (fase 10).
- **Le frontiere non si chiudono**: un raggio del lidar che non colpisce nulla (cielo, orizzonte
  libero) non libera spazio nella mappa, e in simulazione `open_field` ha un recinto alto 5 m e il
  parco una siepe proprio per questo. Rimedio: il **recinto virtuale** (`RECINTO_VIRTUALE=1`, vedi
  il LEGGIMI principale): nelle direzioni senza ritorni si aggiungono alla mappa del pianificatore
  raggi virtuali che liberano lo spazio, senza mai creare ostacoli.
- **La webcam si satura al sole** (foto bianche al 75-93%, mappa colorata al 5%): prima della
  missione `regola_webcam.py` mette l'esposizione manuale e cerca quella giusta. Non ancora
  provato sulla webcam vera: se non accetta i comandi lo dice e resta automatica.

**Prima di andare al campo**
1. Profilo PX4 all'aperto: copia `scheda_sd_aperto/etc/config.txt` (`MPC_XY_VEL_MAX` 1.2,
   `MPC_XY_CRUISE` 1.0) in `/etc/config.txt` della scheda SD, riavvio VERO del Pixhawk (stacca anche
   l'USB). Lasciato sulla scheda, il file riapplica gli stessi valori a ogni avvio; per tornare al
   chiuso va sostituito con `scheda_sd_chiuso/etc/config.txt` (0.6 e 0.5: al chiuso il tetto basso
   limita le correzioni brusche). Il collaudo non puo' leggere i parametri di PX4 (il DDS non li
   porta): la fase 10 chiede conferma. Il lettore di schede va su una porta USB-A del Jetson: quella
   USB-C e' in modo dispositivo e non lo vede.
2. Batteria: se non l'hai gia' fatto, `scheda_sd_una_volta/etc/config.txt` (stima della carica in
   volo, vedi le note del 7 novembre). A 1 m/s per 7 minuti serve una batteria piena; con la stima
   sbagliata il sorvegliante fa atterrare al falso 15%.
3. Prova generale in simulazione (sotto).

**Al campo**: area di 30 x 30 m segnata agli angoli, decollo al centro, e **libero anche 2 m oltre**
(il sorvegliante fa atterrare a 2 m fuori dall'area); niente persone; vento debole; soffitto
nessuno, ma alberi e oggetti alti dentro l'area li evita il pianificatore e quelli bassi (sotto
1 m vicino al drone) il Mid-360 non li vede. **Senza radiocomando** le sole difese sono SPAZIO, il
sorvegliante e il ponte della stima: le prime volte meglio un'area piu' piccola e piu' lenta
(`-- --area-aperto 15 --vel-aperto 0.5`), e un radiocomando con interruttore di spegnimento
collegato al Pixhawk resta fortemente consigliato. Le regole di volo all'aperto (ENAC/EASA) sono a
carico di chi vola.

```bash
cd ros2_ws/scripts/collaudo && FAST_LIO_CONFIG=fast_lio_mid360_aperto.yaml ./avvia_collaudo.sh reale /dev/ttyACM0
```
`fast_lio_mid360_aperto.yaml` = quella da interno con `point_filter_num` 2 (piu' punti) e
`det_range` 60 (superfici lontane): mai provata sul Jetson; la frequenza la controllano la fase 9
e i controlli a terra della missione (almeno 8 Hz). Poi, nella stessa sessione:

- **Fase 9, facoltativa, a piedi, motori spenti**: drone appoggiato al centro dell'area, posizione
  e muso segnati a terra. Sollevalo a ~1 m, vai al bordo, percorri tutto il perimetro, torna al
  centro e riappoggialo sul segno com'era. Passa se FAST-LIO misura un'area di 30 x 30 m (entro il 15%: se
  non vede lo spostamento, qui si scopre), il giro si chiude entro 0.5 m (o lo 0.5% del percorso),
  quota entro 20 cm, imbardata entro 5 gradi, frequenza >= 8 Hz e punti agganciati sufficienti. La
  soglia di atterraggio per la fase 10 diventa il 60% del 5o percentile dei punti agganciati del giro
  (fra 150 e 500), in `collaudo_registri/aperto.json`; lidar, IMU e stima vengono registrati in
  `risultati_prova/<data>_giro_a_piedi/` (servono a tarare FAST-LIO dopo; fuori da git). Vale 12
  ore e solo con la stessa configurazione di FAST-LIO. Dice prima di volare, senza rischiare il
  drone, se la stima regge in quel campo; si puo' saltare (INVIO nel menu propone direttamente la
  10). Se la fai e non passa, la fase 10 chiede se volare lo stesso. Obbligatoria fino al 22
  novembre 2026.
- **Fase 10, la missione**: come la fase 8 (recinto, misure, webcam, `VOLA`, sorvegliante), con
  area 30 x 30 m = recinto della missione, 1 m/s, al massimo 420 s di esplorazione, recinto
  virtuale, margine del sorvegliante 2 m, quota d'allarme 2.6 m (a 1 m/s il drone simulato e'
  salito fino a 2.21 m col riferimento a 2.0), mappa 3D a celle da 5 cm ritagliata all'area piu'
  5 m. Soglia di atterraggio della stima: quella della fase 9 o, senza un giro a piedi valido,
  misurata per 8 s a drone fermo sul decollo dopo la conferma finale (40% del 5o percentile dei
  punti agganciati, fra 150 e 500: con i valori dell'8 novembre 250, contro i 336 del volo
  normale; se gia' da fermo sono meno di 375 chiede se volare). Altri valori: `-- --area-aperto`,
  `--vel-aperto`, `--tempo-aperto`, `--margine-aperto`, `--soglia-aperto` (soglia data a mano).
  In simulazione (campo 30 x 30 senza recinto, 1 m/s) la missione dura ~6.5 minuti in volo,
  compreso circa un minuto fermo sulle ultime frontiere prima di rientrare: sul drone vero serve
  una batteria piena.

Prova generale a casa (campo 50 x 50 m senza recinto, con 10 ostacoli):
```bash
cd ros2_ws && python3 scripts/make_open_field.py /tmp/campo_aperto.sdf --half 25 --senza-recinto --ostacoli 10
cd scripts/collaudo && ./avvia_collaudo.sh sim /tmp/campo_aperto.sdf -- --fase 10 --forza
```
(in simulazione le fasi 9 e 10 non chiedono misure; la missione gira senza recinto vero e con PX4
al tetto del profilo all'aperto).

## Dopo il collaudo: la missione autonoma

Con le fasi 0-7 superate sul drone vero (i controlli a terra della missione lo verificano in
`collaudo_registri/esiti_reale.json`), la missione di esplorazione si avvia con la fase 8 qui
sopra, oppure da sola con
```bash
cd ros2_ws && BOX="-1.5,-2,4,2" ./scripts/run_reale_exploration.sh /dev/ttyACM0
```
(recinto in metri dal punto di decollo, x avanti, y sinistra; 0.3 m/s; SPAZIO = emergenza,
`l` = atterra). Da sola la missione avvia anche agente e driver, accende la webcam se c'e'
(`FOTO`, `CAMERA_DEV`) e usa le misure date nelle variabili (`PARTENZA_Z`, `LIDAR_POS`,
`CAMERA_POS`, `CAMERA_GIU`, `CAMERA_HFOV`: quelle della fase 8 sono in
`collaudo_registri/misure_drone.json`). Prima, a casa: `BOX=... ./scripts/run_reale_exploration.sh sim`.
Tutto, e cosa resta da provare sul drone, nel LEGGIMI principale ("Missione di esplorazione sul
drone vero").

## Cosa NON e' ancora pronto

- La missione autonoma e' pronta ma **mai provata su hardware**: vedi l'elenco nel LEGGIMI
  principale (nuvola di FAST-LIO come mappa, `EKF2_EV_POS_*` a zero, velocita' vera).
- Il Mid-360 vero vede da -7 a +52 gradi in verticale, il simulato da -50 a +50: ostacoli
  bassi vicini restano invisibili (vedi il LEGGIMI principale).
- La telecamera: una webcam USB (`/dev/video0`; il 7 novembre 2026 sul Jetson c'era una
  Logitech UVC 046d:09c1, 640x480 a 30 fotogrammi/s) la avvia `collaudo_reale.launch.py` con
  `cam2image` di image_tools, su `/camera/image_raw`, 640x480 a 10 Hz (`CAMERA_DEV`, `CAMERA=0`
  per non avviarla): la fase 1 ne misura la frequenza. La missione (fase 8, o
  `run_reale_exploration.sh` con `FOTO=1`) avvia un suo `cam2image` a 30 letture al secondo e
  colora la mappa 3D con le sue foto. Mai provato in volo: ottica senza calibrazione (modello a
  foro stenopeico dal campo di vista, distorsione ignorata) e misure della webcam prese a mano;
  si correggono dopo il volo (`ricostruisci_mappa.py --cam_giu ...`).
- La fase 1 conta i messaggi del lidar senza decodificarli: decodificati in Python (~20 000
  punti, ~0.1 s ciascuno) toglievano il tempo all'IMU, che risultava a 9 Hz invece di 200.
