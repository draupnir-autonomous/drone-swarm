# Collaudo del drone vero, passo per passo

Strumenti in questa cartella (ottobre 2026):

| file | cosa fa |
|---|---|
| `PROCEDURA_PROVE_REALI.md` | **la lista di controllo in ordine** (casa, posto, collaudo, missione, dopo il volo), da tenere sul campo |
| `collaudo.py` | il programma del collaudo: 9 fasi in ordine, controlli automatici, levette virtuali da tastiera o joystick, arresto d'emergenza, registro |
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
Non lascia saltare una fase non superata (si puo' con `--forza`, ma resta nel registro).

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
| 8 verso la missione | - | solo istruzioni: cosa manca per il volo autonomo | - |

Tasti in volo: frecce = sposta, `w`/`s` = su/giu', `a`/`d` = ruota, `t` = decolla (in
offboard, con la quota che sale a 0.4 m/s fino a quella della fase, poi resta fermo in
POSCTL; una levetta o `h` interrompe la salita e passa al comando manuale),
`h` = fermo (interrompe anche l'offboard), `l` = atterra, **SPAZIO = emergenza**, `q` = esci
(solo a terra). Nell'offboard della fase 7 una qualsiasi freccia riprende il comando manuale.

Dopo ogni volo: il registro della sessione e' in `collaudo_registri/<data_ora>/`, il
registro di volo di PX4 (`.ulg`) sulla scheda SD. Con il `.ulg` si misura il ritardo vero
della stima (per `EKF2_EV_DELAY`) con lo stesso metodo di `scripts/confronta_stima.py`.

## Dopo il collaudo: la missione autonoma

Con le fasi 0-7 superate sul drone vero (i controlli a terra della missione lo verificano in
`collaudo_registri/esiti_reale.json`), la missione di esplorazione si avvia con
```bash
cd ros2_ws && BOX="-1.5,-2,4,2" ./scripts/run_reale_exploration.sh /dev/ttyACM0
```
(recinto in metri dal punto di decollo, x avanti, y sinistra; 0.3 m/s; SPAZIO = emergenza,
`l` = atterra). Prima, a casa: `BOX=... ./scripts/run_reale_exploration.sh sim`. Tutto, e
cosa resta da provare sul drone, nel LEGGIMI principale ("Missione di esplorazione sul drone
vero").

## Cosa NON e' ancora pronto

- La missione autonoma e' pronta ma **mai provata su hardware**: vedi l'elenco nel LEGGIMI
  principale (nuvola di FAST-LIO come mappa, `EKF2_EV_POS_*` a zero, velocita' vera).
- Il Mid-360 vero vede da -7 a +52 gradi in verticale, il simulato da -50 a +50: ostacoli
  bassi vicini restano invisibili (vedi il LEGGIMI principale).
- La telecamera: una webcam USB (sul drone del 19 ottobre una Logitech C170, `/dev/video0`)
  la avvia `collaudo_reale.launch.py` con `cam2image` di image_tools, su `/camera/image_raw`,
  640x480 a 10 Hz (`CAMERA_DEV`, `CAMERA=0` per non avviarla). Non serve al collaudo: la fase
  1 ne misura solo la frequenza. La missione per ora fa la mappa grigia (la mappa colorata
  vorrebbe la calibrazione della telecamera e la sua posizione rispetto al lidar).
- La fase 1 conta i messaggi del lidar senza decodificarli: decodificati in Python (~20 000
  punti, ~0.1 s ciascuno) toglievano il tempo all'IMU, che risultava a 9 Hz invece di 200.
