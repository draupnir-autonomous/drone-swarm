# Prove sul drone vero: cosa fare, in ordine

Lista di controllo per la campagna di prove sul drone vero (19 ottobre 2026). I dettagli e i
perche' sono in `LEGGIMI_COLLAUDO.md` (collaudo) e nel LEGGIMI principale, sezione "Missione
di esplorazione sul drone vero". Tutti i comandi partono dalla cartella del progetto
`Drone_20261018_P`. Lo stimatore e' FAST-LIO.

## A. Prima, a casa (una volta)

1. **Prova generale in simulazione** (stessi programmi, PX4 simulato):
   ```bash
   cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh sim
   ```
   Fai tutte le fasi 0-7 (INVIO esegue la prossima), cosi' impari tasti e schermate. Poi la
   missione simulata:
   ```bash
   cd ros2_ws && BOX="-1.5,-6,10,6" ./scripts/run_reale_exploration.sh sim
   ```
2. **Parametri PX4**, con QGroundControl e il Pixhawk collegato **al portatile** via USB:
   - Parameters > Tools > Save to file (copia dei parametri attuali);
   - Load from file: `ros2_ws/scripts/collaudo/collaudo_px4.params`, poi riavvio;
   - controlla `MPC_THR_HOVER` = spinta di hover vera del tuo drone (di solito 0.3-0.6);
   - Power: numero di celle (`BAT1_N_CELLS`, era 0) e tensione uguale a quella del tester;
   - airframe giusto, calibrazione di accelerometro e livello, ordine e verso dei motori
     (Vehicle Setup > Actuators).
3. **DDS sul cavo USB**: e' l'**ultimo** passo con QGroundControl, perche' da qui QGroundControl
   non si collega piu' via USB. Solo quando hai finito tutto il punto 2:
   - in QGroundControl imposta `SYS_USB_AUTO` = 0 (Parameters, cerca SYS_USB_AUTO);
   - sulla scheda SD del Pixhawk crea `/etc/extras.txt` con:
     ```
     uxrce_dds_client stop
     uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
     ```
   - riavvia il Pixhawk e collegalo al Jetson.
   **FATTO il 19 ottobre 2026** (Pixhawk 6C, PX4 1.17.0): `extras.txt` c'era gia', mancava
   `SYS_USB_AUTO` = 0 (era 2: PX4 metteva MAVLink sull'USB e la fase 0 diceva "nessun
   vehicle_status"). Impostato dal Jetson; la copia dei parametri di prima e' in
   `collaudo_registri/parametri_px4_prima_usb_auto_0.params`. Provato: sessione DDS aperta,
   stato a 2 Hz, posizione locale a 50 Hz. Se ricapita (per esempio dopo un ripristino dei
   parametri), `avvia_collaudo.sh reale` se ne accorge e propone di sistemarlo da solo
   (`px4_usb_dds.py`).
   Se un giorno il DDS sull'USB non bastasse, la strada
   standard e' un cavo seriale da TELEM2 a un adattatore USB-seriale sul Jetson
   (`UXRCE_DDS_CFG` = 102, `SER_TEL2_BAUD` = 921600, `SYS_USB_AUTO` = 2) e poi `/dev/ttyUSB0`
   al posto di `/dev/ttyACM0` nei comandi.
   **Per tornare a QGroundControl via USB**: sulla scheda SD crea `/etc/config.txt` con
   ```
   param set SYS_USB_AUTO 2
   param save
   ```
   riavvia collegato al PC, e appena QGroundControl si collega cancella `config.txt` (e
   `extras.txt` se non vuoi piu' il DDS sull'USB).
   **`UXRCE_DDS_SYNCT` = 0 (25 ottobre 2026)**: con la sincronizzazione dell'ora attiva, a
   volte (7 avvii su 12) PX4 restava fermo dopo la sessione: i suoi messaggi arrivavano al
   Jetson in ritardo di uno, ogni andata e ritorno sembrava 11 ms invece di <1 ms e PX4 scarta
   i campioni sopra 10 ms. Senza sincronizzazione PX4 crea subito i topic e prende come ora
   di ogni messaggio l'istante di arrivo: i nostri programmi mettono comunque "adesso" e non
   leggono l'ora di PX4. File: `scheda_sd_una_volta/etc/config.txt`, che contiene anche
   `BAT1_V_DIV` = 18.922: il modulo di alimentazione leggeva il 4.5% in meno (16.07 V con la
   batteria carica a 16.8 V) e la carica a schermo risultava 16% col 48% vero.
   **Per cambiare un parametro senza QGroundControl** (DDS sull'USB): stesso `config.txt` con
   `param set NOME VALORE` e `param save` (file con a capo Linux: copia
   `scheda_sd_una_volta/etc/config.txt`, non riscriverlo col Blocco note), poi un riavvio
   VERO: il cavo USB del Jetson tiene acceso il Pixhawk anche senza batteria, quindi stacca
   anche l'USB (o fallo riavviare dal Jetson). Cosi' il 19 ottobre `BAT1_N_CELLS` e' passato
   da 0 (batteria "-100%", fase 0 bloccata) a 4. Poi togli `config.txt`.
4. **Rete del lidar: FATTA il 19 ottobre 2026.** Il lidar del drone e' un **Mid-360S** (tipo
   35, numero di serie ARMCP6H0032822, indirizzo `192.168.1.122`), non un Mid-360: usa
   `MID360s_config.json` e `msg_MID360s_launch.py` (gia' di serie negli script). Sul Jetson
   il profilo di rete **`livox-mid360`** da' alla porta Ethernet `192.168.1.5/24`, senza
   gateway (il traffico internet resta sul Wi-Fi). Provato: nuvola a 10 Hz (~20 000 punti),
   IMU a 200 Hz, gravita' +0.99 g sull'asse z (lidar montato dritto), campo verticale
   misurato -7.2 / +52.7 gradi (come il Mid-360). FAST-LIO sul lidar vero, drone fermo: 10 Hz,
   0.6 cm di deriva in 28 s.
   **Attenzione al Wi-Fi**: con l'hotspot del telefono (192.168.45.x) va tutto bene; se il
   Jetson si collega a una rete Wi-Fi su `192.168.1.x` (quella di casa), il traffico verso
   quella rete passerebbe dall'Ethernet e l'SSH via Wi-Fi si interromperebbe: in quel caso
   `nmcli con down livox-mid360` (e `nmcli con up livox-mid360` per tornare al lidar).
   Per provare il lidar da solo:
   ```bash
   source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360s_launch.py
   ```
   e in un altro terminale `ros2 topic hz /livox/imu` (~200 Hz).
5. **Misure del montaggio** (servono alla missione): `PARTENZA_Z` = quota da terra del punto
   di riferimento del drone appoggiato; `LIDAR_POS` = posizione del Mid-360 rispetto a quel
   punto (x avanti, y sinistra, z su), in metri. Il Mid-360 va montato dritto col davanti
   verso il muso (la fase 3 lo controlla).
6. **Jetson pronto per il Pixhawk sull'USB**: spegni ModemManager, che interroga le porte
   `/dev/ttyACM*` come se fossero modem e disturba il collegamento:
   ```bash
   sudo systemctl disable --now ModemManager
   ```
   Collegato il Pixhawk, deve comparire `/dev/ttyACM0` (`ls /dev/ttyACM*`). Il tuo utente
   e' gia' nel gruppo `dialout`, quindi i permessi ci sono.
7. **SSH col controllo del collegamento** (se il Wi-Fi cade, la sessione si chiude in pochi
   secondi e i programmi fanno atterrare il drone): dal portatile
   ```bash
   ssh -o ServerAliveInterval=1 -o ServerAliveCountMax=3 utente@jetson
   ```
   e sul Jetson in `/etc/ssh/sshd_config`: `ClientAliveInterval 1`, `ClientAliveCountMax 3`.

## B. Sul posto, prima di volare

- Locale **con struttura vicina** (pareti, colonne, scaffali o scatoloni entro ~10 m in ogni
  direzione: in una stanza vuota FAST-LIO puo' non vedere gli spostamenti), soffitto di
  almeno **2.8 m**, spazio sgombro **sotto 1 m** (il Mid-360 non vede gli ostacoli bassi
  vicini), protezioni delle eliche, una **corda** per la fase 5, nessuno entro 3 m.
- Batteria > 15% (sotto, il collaudo non parte e in volo atterra), eliche e viti a posto, lidar pulito, cavi fissati.
- **Pixhawk acceso da zero a ogni sessione di prove**: se il Jetson si e' riavviato, se hai
  staccato il cavo USB o tolto la scheda SD (mai col Pixhawk acceso), spegnilo del tutto
  (batteria E USB) e riaccendilo. La batteria lo tiene acceso anche senza USB, e il 21
  ottobre dopo ore cosi' il DDS si e' bloccato (sessione aperta, nessun topic: fase 0
  "nessun vehicle_status"). `avvia_collaudo.sh` ora se ne accorge e lo dice.
- Collega: Pixhawk al Jetson col cavo USB, Mid-360S alla porta Ethernet, batteria. Dal
  portatile entra nel Jetson con l'SSH del punto A.6.
- **Guarda il drone, non lo schermo.** SPAZIO = motori spenti subito (il drone cade),
  `l` = atterra. Se il drone si sposta da solo mentre lo schermo dice fermo, la stima non
  vede il movimento: `l` se e' lento e lontano dagli ostacoli, altrimenti SPAZIO.

## C. Collaudo, fasi 0-7 (il primo giorno, e dopo ogni modifica al drone)

```bash
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh reale /dev/ttyACM0
```
(con un gamepad: aggiungi `-- --joystick /dev/input/js0`). INVIO esegue la prossima fase.

| fase | eliche | cosa fai |
|---|---|---|
| 0 | no | rispondi alle domande; PX4 deve accettare le levette virtuali |
| 1 | no | niente: misura lidar, IMU, telecamera |
| 2 | no | drone fermo a terra 30 s, nessuno lo tocca |
| 3 | no | **nel punto di decollo**: inclini il drone avanti e a destra, lo sposti avanti di 50 cm, lo ruoti a sinistra di 90 gradi (segui le istruzioni) |
| 4 | **no** | scrivi SENZA ELICHE; controlli posizione e verso di ogni motore; prova dell'emergenza |
| 5 | si | **drone legato**: `t` decolla a 0.5 m, resta fermo 30 s, `l` atterra |
| 6 | si | `t` a 1 m, frecce per piccoli spostamenti, `w`/`s` su/giu', `a`/`d` ruota, `l` atterra |
| 7 | si | `t`, poi `o`: quadrato automatico di 1 m; `l` atterra |

Se una fase non passa, il programma dice perche' e non lascia andare avanti: risolvi e
ripetila. Le fasi superate sul drone vero restano scritte (`collaudo_registri/esiti_reale.json`).

## D. Missione di esplorazione (solo dopo le fasi 0-7 superate)

Drone appoggiato **nel punto di decollo**, col muso dove vuoi l'asse x. Prima missione:
recinto piccolo, 4-5 m per lato, con il decollo ad almeno 1 m dai bordi.
```bash
cd ros2_ws && BOX="-1.5,-2,4,2" PARTENZA_Z=0.20 LIDAR_POS="0,0,0.10" ./scripts/run_reale_exploration.sh /dev/ttyACM0
```
(metti in `PARTENZA_Z` e `LIDAR_POS` le tue misure del punto A.5; `BOX` = x min, y min,
x max, y max in metri dal punto di decollo, x avanti, y a sinistra). Lo script fa i controlli
a terra, poi chiede di scrivere `VOLA`: da li' il drone **arma e decolla subito** a 1 m, gira
su se stesso, esplora a 0.3 m/s per al massimo 240 s, rientra e atterra sul punto di
decollo. Durante tutto il volo resta aperta la schermata del sorvegliante: SPAZIO = motori
spenti, `l` = atterra; atterra da solo se esce dal recinto, se la stima si ferma o peggiora,
se la batteria scende sotto il 15%, se scade il tempo o se cade l'SSH.

Alla prima accensione della stima guarda in RViz la nuvola di FAST-LIO
(`/drone_1/lidar/points`): non devono esserci punti sul telaio o sulle eliche.

Il ponte dei comandi smette di pubblicare quando PX4 esce da solo dall'offboard (failsafe,
atterraggio comandato da fuori): corretto e provato in simulazione il 19 ottobre (run di
controllo nel parco pulita; NAV_LAND da fuori: a terra in 2.2 s, disarmato in 5.2 s), mai sul
drone vero.

## E. Dopo ogni volo

- Registri: `ros2_ws/scripts/collaudo/collaudo_registri/` (collaudo), `/tmp/c2x_run_reale` e
  `risultati_prova/<data_ora>_reale_lio_v0.3/` (missione). Il registro di volo di PX4 (`.ulg`)
  resta sulla scheda SD del Pixhawk: copialo, serve a misurare il ritardo vero della stima
  (`EKF2_EV_DELAY`) e a capire qualsiasi cosa strana.
- Se tutto torna, allarga a poco a poco: recinti piu' grandi, poi `VEL_MAX` piu' alto.
