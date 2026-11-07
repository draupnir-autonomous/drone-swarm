# C²-Explorer su ROS2 + PX4 + Gazebo

Esplorazione autonoma multi-drone, portata su ROS2 / PX4 v1.17 / Gazebo Harmonic.
Gira su **Ubuntu 24.04 + ROS2 Jazzy** (sviluppato in origine su 22.04 + Humble).
Include una modalità di **volo senza GPS** in cui la posizione dell'autopilota viene
interamente da lidar e IMU tramite FAST-LIO2.

**Questa cartella (Drone_20261018_P)** e' la copia di lavoro snella di `Drone_20261017_P`,
creata il 18 ottobre 2026 per le prove successive (FAST-LIVO2 e mappa 3D a bordo). Contiene
solo cio' che serve a volare e a sviluppare: PX4 (sorgenti, storia git, build), i sorgenti
e gli script di `ros2_ws` (ricompilati qui da zero), l'agente DDS, gli script di
installazione. Restano solo in `Drone_20261017_P`: l'archivio delle run
(`risultati_prova/`), il tentativo con Glob3R (`glob3r/`), l'implementazione di riferimento
del paper (`reference/`), il paper (`C2.pdf`), la roadmap e i log di compilazione.

---

## Cosa c'è in questo archivio

```
ros2_ws/src/          il codice: port di C2-Explorer + i ponti verso PX4
ros2_ws/scripts/      script di avvio, verifica e campagne di prova
PX4-Autopilot/        PX4 con l'airframe 22000 personalizzato
Micro-XRCE-DDS-Agent/ ponte DDS fra ROS2 e PX4
reference/            implementazione di riferimento del paper C2-Explorer
```

**Non è incluso** quello che si rigenera o si riscarica: cartelle `build/`,
`install/`, `log/`, le `.git/` e i registri di volo `.ulg`. Vedi "Come ripartire".

---

## I pezzi scritti per questo progetto

Il resto è codice di terze parti (C2-Explorer, FAST-LIO, PX4). Questi invece sono
i componenti sviluppati qui, ed è dove guardare per capire il sistema:

| percorso | cosa fa |
|---|---|
| `ros2_ws/src/c2x_px4_bridge/src/odometry_bridge.cpp` | posa da PX4 **o** da FAST-LIO verso lo stack |
| `ros2_ws/src/c2x_px4_bridge/src/lio_to_px4_bridge.cpp` | porta la stima di FAST-LIO dentro EKF2 come visione esterna |
| `ros2_ws/src/c2x_px4_bridge/src/lidar_lio_adapter.cpp` | adatta la nuvola di Gazebo al formato che FAST-LIO si aspetta |
| `ros2_ws/src/c2x_px4_bridge/src/position_command_bridge.cpp` | traiettorie verso PX4 + strato reattivo anticollisione |
| `ros2_ws/src/c2x_px4_bridge/src/cloud_transform_bridge.cpp` | nuvola lidar in sistema mondo |
| `ros2_ws/src/exploration_manager/` | macchina a stati, allocazione fra droni |
| `ros2_ws/scripts/run_multi_exploration.sh` | la missione, su qualsiasi mondo: avvio, volo, mappa 3D, archivio |
| `ros2_ws/scripts/prepara_mondo.py` + `geometria_mondo.py` | prepara un mondo `.sdf` qualsiasi: zona da esplorare, partenza, recinto, configurazione |
| `ros2_ws/scripts/run_reale_exploration.sh` + `scripts/collaudo/sorveglia_missione.py` | la stessa missione sul **drone vero** (Mid-360, Pixhawk via USB), con controlli a terra, consenso e sorvegliante con arresto d'emergenza |
| `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_x500_mid360` | airframe, con tutti i parametri del volo cieco |

I file sono commentati fitto, **comprese le modifiche provate e scartate con i
numeri che le hanno bocciate**. Quei commenti valgono quanto il codice: evitano di
rifare esperimenti già falliti.

---

## Come ripartire da zero (Ubuntu 24.04)

Tutti i percorsi del progetto sono **relativi**: la cartella si puo' spostare o
copiare, anche con le build dentro, e le missioni partono senza toccare nulla (vedi
"Progetto trasportabile" sotto). La copia va fatta conservando le date dei file
(`cp -a` o `rsync -a`): il controllo prima di ogni run confronta le date dei
sorgenti con quella dell'ultima compilazione. Su una macchina di ARCHITETTURA
diversa (PC x86 contro Jetson aarch64) le build vanno rifatte: `rm -rf
ros2_ws/build ros2_ws/install ros2_ws/log PX4-Autopilot/build` e poi i passi 2-4.

```bash
# 1. dipendenze di sistema (ROS2 Jazzy, Gazebo Harmonic, dipendenze PX4) - chiede sudo
bash setup_jazzy.sh

# 2. PX4 (v1.17 con i sottomoduli)
cd PX4-Autopilot && make px4_sitl_default && cd ..

# 3. agente DDS, installato in ~/.local senza sudo
cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local && make -j16 && make install && cd ../..

# 4. spazio di lavoro ROS2
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

L'agente ha bisogno delle sue librerie in `~/.local/lib`: va avviato tramite un
piccolo wrapper `~/.local/bin/MicroXRCEAgent` che imposta `LD_LIBRARY_PATH`
(gli script aggiungono `~/.local/bin` al PATH da soli).

FAST-LIO ha un sottomodulo (`ikd-Tree`): se la compilazione si lamenta di
`ikd_Tree.cpp` mancante, va recuperato con `git submodule update --init --recursive`.

### Su Jetson Orin Nano Super (Ubuntu 24.04, aarch64) — ottobre 2026

Stesse modifiche del primo porting su Jetson (`Porting_Jetson_changes.pdf`). Nessun
algoritmo, parametro, airframe, modello o mondo cambiato.

- sistema: `./setup_jetson_jazzy.sh` al posto di `setup_jazzy.sh` (repository OSRF,
  locale, swap da 8 GB); a parte `libswscale-dev libavdevice-dev ...` (dipendenza
  mancante di gz-common5-av su arm64), MAXN_SUPER (`sudo nvpmodel -m 2`) e l'agente
  DDS, qui in `/usr/local/bin` (il wrapper in `~/.local/bin` non serve)
- `PX4-Autopilot/.git`: l'archivio aveva solo uno scheletro senza oggetti; ripristinato
  (stesso commit v1.17.0), `git status` mostra solo le modifiche del progetto. Anche
  `Micro-XRCE-DDS-Agent` ricollegato a git (v3.0.1, identico)
- `Tools/simulation/gz/models/x500_mid360` mancava: ora e' un collegamento relativo a
  `ros2_ws/src/c2x_description/models/x500_mid360`, come i 4 mondi. Ripristinati da git
  anche i file di pymavlink `generator/javascript/node_modules`, e ricreata `fast_lio/Log/`
- `px4_ros_com` portato a C++17; OpenMP di FAST-LIO attivo anche su aarch64 (3 thread);
  `find_package(NLopt ...)` con `HINTS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/...`
  invece del percorso fisso x86_64
- `fastdds_profilo.xml`: 32 MB di memoria condivisa e messaggi fino a 4 MB, i valori
  validati sul Jetson; ora lo carica anche `run_exploration.sh`. Senza, sul Jetson
  FAST-LIO riceveva ~3 scansioni/s su 10 e il drone senza GPS finiva nel muro
- Open3D 0.20.0 installato per l'utente (`pip install --user --break-system-packages open3d`)
- `ricostruisci_mappa.py`: sul Jetson Poisson multi-thread abortiva sempre ("Failed to close
  loop") e la mappa 3D non usciva mai: su aarch64 gira a un thread (cubicle_office: 147 s)
- rendering fuori schermo di Open3D (`video_demo.py`, `video_replay3d.py`; le anteprime della mappa
  3D dal 17 ottobre usano il visualizzatore OpenGL, vedi sotto): sul
  Jetson (Filament su Vulkan) la mesh intera della mappa (1.2 M triangoli) e le scene pesanti
  a 1080p escono tutte nere. Su aarch64 si rende a 1280x720 (ingrandito a 1080p nei video) con
  la mappa ridotta a 250k triangoli e il modello X500 a 40k (intero, 210k, il 5% dei fotogrammi
  del replay usciva nero); un fotogramma ancora nero viene sostituito dal precedente. Nel
  replay la traiettoria si aggiunge a pezzi fissi da 10 s e si ricarica solo la coda: ricaricarla
  tutta a ogni fotogramma faceva fallire sempre piu' fotogrammi (volo di 16 min: video a scatti)
- **cubicle_office senza GPS: usare `VEL_MAX=0.7`** sul Jetson. Due run a 1.0 m/s (14 ottobre)
  sono finite contro un ostacolo dopo ~500 s (urto vero: FAST-LIO entro 0.15 m fino all'impatto,
  diverge solo dopo); a 0.7 m/s run pulita. Con la GUI accesa la pianificazione arrivava a 3.9 s
- un volo su cubicle_office porta la RAM a ~0.5 GB liberi (`registra_mappa.py` ~1.8 GB): lo
  swap da 8 GB serve. Se il drone cade, la mappa 3D della run e' da buttare (i punti registrati
  dopo la divergenza la sfasciano)

Video senza GUI di Gazebo, da una run archiviata (`rel_ulog_1_0.ulg`, `rel_expl_1.log`):

```bash
cd ros2_ws/scripts && python3 video_replay3d.py ../../risultati_prova/<cartella> 1 ../../risultati_prova/<cartella>/replay3d.mp4 --mondo cubicle_office --fps 30
```

Dopo ogni riavvio: `nvpmodel -q` deve dire MAXN_SUPER, poi `sudo jetson_clocks`.
Rispondere N alla GUI di Gazebo e non aprire RViz. Salute della stima senza GPS in
`/tmp/c2x_run_multi/exploration.log`: la riga `visione esterna verso PX4: N pose inviate`
(ogni 5 s) deve crescere di ~50 ogni volta.

#### Velocita' sul Jetson: cubicle_office nei tempi del PC (17 ottobre 2026)

Prova di riferimento: `ODOM_SOURCE=lio VEL_MAX=1.0 VISITA=0`, cubicle_office, un drone. Sul PC
(V31) esplorazione 277-283 s, missione (decollo-appoggio) 313-342 s. Sul Jetson, prima: esplorazione
496-613 s, missione ~535-645 s. Dopo: esplorazione **274-328 s**, missione **297-368 s**.

Cosa e' stato misurato (`gdb` a campioni sul nodo di esplorazione, `top`, `/stats` di Gazebo):
- pianificazione, ESDF, fusione delle nuvole e pubblicazione delle mappe per RViz stanno nello
  STESSO gruppo di callback (seriale) di `exploration_node`, che sul Jetson era occupato al 100%:
  ESDF 0.12 s a nuvola, mappe RViz 0.035-0.058 s ogni 0.5 s (senza RViz aperto), e il resto della
  pianificazione per l'80% dentro `Astar::search` (grafo di connettivita', matrici dei costi)
- Gazebo in volo a RTF 0.87; Gazebo + PX4 da soli a RTF 1.000: e' la CPU contesa (libera ~12%),
  non Gazebo. **L'app Claude Desktop aperta costa ~70-85% di un core e gnome-shell ~40-45%**: per
  le misure tenerla chiusa o ridotta a icona (tutte le run qui sopra sono state fatte con l'app aperta,
  RTF 0.89)

Modifiche (commentate nel codice con le misure):

| dove | cosa |
|---|---|
| `sdf_map.cpp` (`clearAndInflateLocalMap`) | gonfiaggio separabile (z, poi y, poi x) invece del cubo 7x7x7 per voxel occupato: risultato identico (provato su dati casuali) |
| `sdf_map.cpp` + `multi_drone_cubicle_office.yaml` | ESDF solo entro `sdf_map.esdf_radius` = 10 m dal drone (lo legge solo l'ottimizzatore, tratti <= 7 m) e **su richiesta prima di ogni pianificazione** (`SDFMap::aggiornaESDFSeServe`), col timer di riserva a `map_ros.esdf_period` = 1 s: da 1458 a ~600 calcoli per volo |
| `map_ros.cpp` | le nuvole per RViz si costruiscono solo se qualcuno e' iscritto |
| `astar2.cpp` | insiemi aperto/chiuso su array densi con contatore di generazione al posto delle `unordered_map`, un lock della mappa gonfiata per nodo espanso invece di ~130, `steady_clock`. Stessi percorsi, solo piu' nodi esplorati nello stesso tempo |
| launch | grafo di connettivita' su 3 thread (`CG_THREADS`, di serie 3): 0.20-0.24 s invece di 0.39 |
| `c2_exploration_fsm.cpp` | **due urti** su cubicle_office a 1.0 m/s venivano da piani arrivati tardi: (1) un piano dallo stato misurato arrivato dopo 6.6 s veniva preso a 2.8 m dal drone (ora scartato oltre 0.5 m); (2) un piano dalla traiettoria preso "gia' avviato" con 0.7 s di ritardo aveva la posizione giusta ma la velocita' invertita (+1.1 -> -0.2 m/s): ora si scarta anche oltre 0.5 m/s di salto di velocita' |
| `run_multi_exploration.sh`, `registra_mappa.py` | registratore della mappa 3D a `nice 10` e senza iscrizione a `/clock` quando le foto sono spente: registra comunque tutte le scansioni (3153 in un volo da ~315 s) |
| `exploration_node.cpp` | `C2X_PROFILO=1` nell'ambiente permette a `gdb -p` di agganciarsi (ptrace_scope=1) per campionare gli stack |

Run del 17 ottobre (archiviate in `risultati_prova/2026-10-17_*`, `analisi_campagna.py`):

| run | codice | esplorazione | missione | margine vero | esito |
|---|---|---|---|---|---|
| 1113 | di partenza | 613 s | 599 s | **0.27 m** | conclusa, contatto |
| 1135 | di partenza | - | - | - | urto a ~280 s |
| 1153 | mappa + ESDF a 10 m | - | - | - | urto a 67 s (piano dallo stato misurato preso a 2.8 m) |
| 1223 | + scarto piani spostati + A* | 381 s | 399 s | 0.50 m | pulita |
| 1238 | + ESDF ogni 0.25 s, 3 thread | - | - | - | urto a ~150 s (piano con velocita' invertita) |
| 1251 | + scarto per salto di velocita', nice | 328 s | 344 s | **0.30 m** | conclusa, contatto |
| 1308 | + ESDF su richiesta | 278 s | 332 s | 0.47 m | pulita |
| 1319 | idem | 274 s | 297 s | 0.40 m | pulita |
| 1336 | codice finale, comando normale | 323 s | 368 s | 0.46 m | pulita |

Velocita' vera massima 1.7-2.3 m/s col tetto a 1.0. Per un giudizio sull'affidabilita' a 1.0 m/s
servono piu' run (`reliability_campaign.sh`): finora 4 pulite + 1 contatto con le due correzioni
(anche la run col codice di partenza aveva un contatto, a 0.27 m).

**Sicurezza anti-urto (17 ottobre 2026, pomeriggio).** Analisi dei contatti e quasi-contatti
(posizione VERA di Gazebo contro la geometria del mondo, `eventi_vicini` nello stile di
`analisi_campagna.py`): in tutti, un secondo prima il drone puntava l'ostacolo a 0.8-1.3 m/s mentre
il riferimento ne stava a 0.6-1.0 m. Inseguendo il riferimento con 0.4-1.0 m di ritardo (dopo
un'inversione di marcia, o col RTF < 1 che fa scorrere la traiettoria in tempo reale e il drone in
tempo simulato) PX4 aggiungeva la correzione di posizione all'anticipo e chiedeva 1.43-1.48 m/s,
il drone ne toccava 1.6-1.9 veri e andava lungo verso la parete. La stima di velocita' di PX4 NON e'
in ritardo (0 ms, rms 0.10 m/s, uguale al PC); i picchi sono transitori dell'anello di velocita'.

| protezione | dove | esito |
|---|---|---|
| tetto di velocita' PX4 senza GPS 1.5 -> **1.2 m/s** | airframe 22000 (copiato anche in `build/.../etc`) | tenuto: il piano non supera 1.13 m/s, il tetto taglia solo il "recupero" |
| **limite di frenata** sugli ostacoli lidar nel tubo di 0.45 m lungo la velocita' (1.5 m/s2, 0.3 s di ritardo): se il drone non puo' fermarsi prima, il riferimento non resta davanti e l'anticipo scende al valore ammesso | `position_command_bridge.cpp` (`enable_brake_governor`), acceso nel launch | tenuto |
| velocita' ridotta vicino agli ostacoli (0.5 m/s a 0.6 m, 1.2 a 0.9 m) | stesso ponte (`enable_proximity_speed`) | **scartato**: margini 0.35-0.55 m e missioni piu' lente (mediana 333 s) |
| frenata attiva (accelerazione contraria al moto durante il limite) | stesso ponte (`brake_gov_acc_gain`) | **scartato**: 2 urti su 5 nei passaggi stretti (accelerazione comandata che oscilla a +-2 m/s2) |

Con tetto 1.2 + limite di frenata + scarto dei piani tardivi, 8 run su cubicle_office senza GPS a
1.0 m/s: 8/8 concluse e atterrate, 0 urti, 0 contatti, margine vero minimo 0.35-0.69 m (mediana
0.48), missione 281-332 s (mediana 306), riferimento mai sotto 0.55 m dagli ostacoli. Prima delle
protezioni (stessa mattina, 10 run con versioni intermedie del codice): 3 urti e 2 contatti (0.27 e
0.30 m). Il margine restava a 0.35-0.40 m in due run su otto, sempre per lo scarto fra drone e
riferimento (fino a 1 m) dopo un'inversione di marcia.

**Scarto drone-riferimento (17 ottobre 2026, sera).** La stima di velocita' di EKF2 non e' in
ritardo (la velocita' derivata che il ponte manda a PX4 lo e' di ~170 ms, ma EKF2 la compensa con
l'IMU) e l'anticipo di accelerazione non spinge oltre (e' negativo quando il drone va piu' forte del
comando). Il collo di bottiglia e' l'anello di velocita' di PX4, che col guadagno di serie frena
piano. Provati uno alla volta, 5 run ciascuno (`scarto` = stima PX4 contro trajectory_setpoint):

| | di partenza (8 run) | `max_setpoint_jump` 0.65 | 0.65 + `MPC_XY_VEL_P_ACC` 2.5 |
|---|---|---|---|
| scarto mediano / 99% | 0.20 / 0.70 m | 0.18 / 0.59 m | **0.09 / 0.37 m** |
| scarto massimo (media) | 0.94 m | 0.71 m | **0.58 m** |
| tempo con scarto > 0.5 m | 3.6% | 2.4% | **0.4%** |
| tempo sopra 1.3 m/s veri | 8.7% | 8.6% | **5.4%** |
| margine vero minimo | 0.35-0.69 m | 0.41-0.64 m | **0.50-0.75 m** |
| missione (media) | 305 s | 306 s | 327 s (percorsi piu' lunghi in 2 run: velocita' media invariata, 0.90-0.97 m/s) |
| esito | 8/8 pulite | 5/5 pulite | 5/5 pulite |

Tenuti entrambi, di serie: `max_setpoint_jump` 0.65 nel launch (`SALTO_RIFERIMENTO` lo cambia) e
`MPC_XY_VEL_P_ACC` 2.5 nel ramo senza GPS dell'airframe 22000 (`PX4_VEL_P_ACC` lo cambia; il file va
ricopiato anche in `PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/`). Run di
verifica col comando normale (2059): scarto 0.09 / 0.34 m, margine 0.60 m, missione 297 s. Totale con
la configurazione finale: 6 run, 0 urti, margine minimo 0.50 m. Inclinazione massima invariata
(16-18 gradi) e scarto fra velocita' comandata e vera sceso da 0.20 a 0.13 m/s: nessun segno di
oscillazione. Sul drone vero le prime prove vanno fatte comunque con `VEL_MAX` ridotto, e il
guadagno 2.5 va verificato sul velivolo reale (inerzia e ritardi diversi da quelli simulati).

**Telecamera solo per il colore della mappa 3D** (17 ottobre 2026, `FOTO=1` di serie). Il plugin
`GstCameraSystem` di PX4 (in `PX4-Autopilot/src/modules/simulation/gz_bridge/server.config`) si
iscriveva alla telecamera frontale e la trasmetteva come video H.264 su UDP 5600, codificato in
software (x264, 4 thread): la telecamera era resa e codificata in TUTTE le run, anche con `FOTO=0`.
Tolto dal file (commentato). Ora la telecamera la usa solo `registra_mappa.py` (ponte avviato da
lui stesso), senza fase fotografica, imbardata da foto o video. Run di prova (1411): esplorazione
308 s, missione 346 s, margine 0.55 m, 706 fotogrammi chiave, 90% della superficie colorata. Il
mondo cubicle_office in Gazebo e' in toni di grigio, quindi anche la mappa colorata lo e'
(pavimento chiaro, pareti scure).

**Anteprime della mappa 3D sul Jetson.** `anteprima_alto.png` e `anteprima_prospettiva.png` uscivano
tutte nere in ogni run (renderer fuori schermo Filament/Vulkan, anche con la mesh ridotta a 250k
triangoli). Ora `ricostruisci_mappa.py` su aarch64 le disegna col visualizzatore OpenGL classico di
Open3D, lo stesso della finestra a schermo (`mostra_mappa.py`), in una finestra nascosta: mesh intera,
1600x1200, ~1.3 s. Senza display, o se l'immagine esce vuota, ripiega sul renderer di prima. Le
anteprime nere gia' archiviate (run del 17 ottobre e `JETSON_cubicle_lio_v07`) sono state rifatte
dalle loro `mappa_realistica.ply`.

#### Scenario `parco`: all'aperto, mai visto (17-18 ottobre 2026)

```bash
cd ros2_ws && ODOM_SOURCE=lio VEL_MAX=1.0 VISITA=0 ./scripts/run_multi_exploration.sh parco 1
```

Parco di quartiere 31 x 31 m (961 m2), generato da `scripts/make_parco.py` con modelli ufficiali
del catalogo Gazebo Fuel copiati in `src/c2x_worlds/models_parco/` (licenze CC0, il cestino CC BY:
vedi `LICENZE.txt`; il mondo gira senza rete). A sud una strada asfaltata con SUV e due utilitarie
parcheggiate, marciapiede con lampioni, idrante, cassetta postale e stop, e oltre la strada due case;
dentro, area giochi, fontana con panchine e cestini, 3 querce e 4 pini; siepe alta 4 m su tre lati e
una casa oltre la siepe nord. Il confine serve come il recinto di `open_field`: senza, i raggi
orizzontali del lidar non tornano e l'esplorazione non si chiude. Configurazione
`multi_drone_parco.yaml` = quella di cubicle_office con la box ridotta all'area e tetto a 2.2 m.
Il generatore scrive anche `parco_ostacoli.npz` (pianta VERA degli ostacoli fra 0.4 e 1.9 m dalle
collisioni dei modelli, letta da `analisi_campagna.py --mondo parco`) e `parco_pianta.png`
(verde = raggiungibile dal drone: 641 m2 su 642 liberi). La mappa lidar di una run coincide con
la pianta (87% dei punti entro 15 cm).

Prima di comporlo si sono cercati mondi pronti: nessuno era insieme all'aperto, realistico, entro
1000 m2 e volabile fra 1.0 e 1.7 m. Il frutteto di Clearpath (olivi con chiome fra 0.4 e 2.0 m)
lascia fra i filari 0.34-0.50 m liberi dal centro, contro i 0.65 che servono; il bosco di PX4
(`forest.sdf`) e' 1726 m2 di soli alberi; Harmonic World, Baylands e Nathan Benderson Park sono
troppo grandi o pesanti.

**Prime prove (17 ottobre): 3 run su 3 finite con un urto, per DERIVA DI FAST-LIO.** Con la
stessa configurazione che in ufficio tiene l'errore a +-2 cm in quota e 4-10 cm in orizzontale, nel
parco l'errore cresceva entro 20-60 s: orizzontale fino a 0.3-2.5 m e poi divergente, oppure in
quota (il drone si credeva 0.5-1.7 m piu' alto, volava a 0.3 m e toccava terra). Anche senza alberi
(`PARCO_SENZA_ALBERI=1 python3 scripts/make_parco.py`). Il rilevatore di divergenza del ponte (3
campioni di fila oltre 5 m/s) scattava solo dopo l'urto. Run in `risultati_prova/2026-10-17_2345_parco*`,
`..._2354_parco*`, `..._2359_parco*`.

**Causa (18 ottobre): i raggi del lidar che vanno verso il cielo.** Il lidar simulato (300 x 64)
mette a +inf le coordinate dei raggi senza ritorno: all'aperto sono ~2850 su 19200 per scansione, in
ufficio 0 (c'e' il soffitto). FAST-LIO scarta solo i punti troppo vicini (`blind`), quindi gli +inf
arrivavano al filtro a voxel di PCL, che col riquadro dei punti infinito sbaglia gli indici delle
celle: la scansione crollava da ~6200 punti a ~273, di cui ~140 agganciati a un piano della mappa
invece di ~2800. Misurato registrando un volo (lidar, IMU e verita' di Gazebo in un bag) e
rifacendo girare FAST-LIO sugli stessi dati: com'era, stessa deriva; togliendo i soli punti non
finiti, errore 1-5 cm fino all'urto vero della registrazione. **Correzione** in
`lidar_lio_adapter.cpp`: passano a FAST-LIO solo i punti finiti (l'anello si ricava dalla riga
originale prima di compattare). Un lidar vero non manda +inf, ma il filtro non costa nulla.

**Rilevamento piu' pronto.** FAST-LIO pubblica a ogni scansione `/drone_N/lio/salute` (punti dopo il
filtro, punti agganciati, autovalore minimo l1 dei vincoli; `laserMapping.cpp`). `lio_to_px4_bridge`
chiede l'atterraggio sul posto e chiude la missione se i punti agganciati restano **sotto 500 per 5
scansioni di fila** (0.5 s); la stima continua ad andare a PX4 (degradata, non persa), e a terra la
stessa regola impedisce il decollo. `SALUTE_MIN_AGGANCIATI=0` la spegne. Punti agganciati per
scansione:

| caso | min | mediana | max |
|---|---|---|---|
| parco col difetto (+inf), rigiocato | 0 | 141 | 239 |
| parco sano, 4 voli | 1329 | 2180-2360 | 3435 |
| ufficio (`cubicle_office`), 2 voli | 993 | 1680-1710 | 3034 |

Sul volo guasto rigiocato la regola scatta alla quinta scansione di FAST-LIO, a terra; sul volo sano
rigiocato non scatta mai prima dell'urto registrato (dopo, scatta prima il rilevatore di
divergenza). **Non si usa l1**: la prima versione (l1 < 100) ha fatto atterrare un volo d'ufficio
sano al decollo, perche' l'ufficio vincola poco una direzione (l1 82-90 per minuti a terra, 69 in volo,
contro 28 di mediana nel parco guasto). Non e' servito un sensore di distanza verso il basso: tolto il difetto,
l'errore di quota resta sotto 4 cm (p95). `C2X_LIO_DIAG=<file.csv>` nell'ambiente fa scrivere a
FAST-LIO il dettaglio di ogni scansione.

**Verifica (18 ottobre, Jetson, comando qui sopra; ufficio con il comando solito).** Errore della
stima mandata a PX4 contro la verita', in volo; margine = distanza minima VERA in 3D dagli ostacoli.

| run | esito | esplorazione | missione | margine 3D | errore xy mediana / p95 / max | quota p95 |
|---|---|---|---|---|---|---|
| parco `0054` (solo correzione) | finita | 265 s | 287 s | 0.53 m | 0.05 / 0.13 / 0.22 m | 0.04 m |
| parco `0137` | finita | 285 s | 323 s | 0.48 m | 0.04 / 0.12 / 0.21 m | 0.03 m |
| parco `0146` | finita | 270 s | 309 s | 0.31 m (*) | 0.04 / 0.13 / 0.22 m | 0.04 m |
| parco `0155` | **urto** a 155 s (**) | - | - | - | 0.03 / 0.13 m fino all'urto | 0.03 m |
| parco `0213` | finita | 178 s | 228 s | 0.49 m | 0.07 / 0.13 / 0.20 m | 0.04 m |
| parco `0220` | finita | 159 s | 221 s | 0.51 m | 0.07 / 0.13 / 0.21 m | 0.04 m |
| ufficio `0121` | finita | 263 s | 290 s | 0.50 m | 0.07 / 0.14 / 0.21 m | 0.04 m |
| ufficio `0200` | finita | 268 s | 293 s | 0.58 m | 0.07 / 0.13 / 0.25 m | 0.03 m |

Run in `risultati_prova/2026-10-18_<ora>_*`. FAST-LIO non ha mai deviato in nessun volo e
l'allarme di stima degradata non e' mai scattato in volo. I due incidenti NON vengono dalla stima:

- (**) **Rami invisibili al lidar**, risolto subito dopo (vedi sotto). Nel punto d'urto (ramo basso
  del pino a nord-est, 1.6 m dal tronco) la collisione aveva 61 punti entro 0.3 m, la mappa 0;
  errore di stima fino all'impatto <= 0.12 m.
- (*) Passaggio a 0.31 m dall'area giochi, che era in mappa: e' lo scarto drone-riferimento gia'
  visto in ufficio, non la stima (inclinazione max 19 gradi, nessun urto).

**Ostacoli invisibili al lidar (18 ottobre 2026, risolto).** Il lidar simulato non vedeva tutto
cio' con cui il drone puo' urtare. Rispetta la trasparenza delle TEXTURE: le chiome di pini e querce
del catalogo sono rettangoli a texture trasparente. E scarta le FACCE POSTERIORI dei triangoli anche
con `double_sided`. La collisione invece e' la mesh intera, nei due versi. Misurato con
`scripts/verifica_lidar_parco.py`: lidar fermi identici a quello del drone sparsi nel parco a quota di
volo, ogni raggio confrontato con un raycast sulla geometria di collisione.

| pezzo | visto dal lidar, prima | dopo |
|---|---|---|
| chiome dei pini (rami) | 15% | 86-94% |
| chiome delle querce (rami) | 20% | 93-97% |
| facciata della casa a sud-ovest (solo facce posteriori verso il parco) | 1% | 95-98% |
| tutto il resto (case, auto, panchine, fontana, siepi, marciapiede...) | 73-100% (i piu' bassi sono pali sottili) | invariato |

Della facciata di quella casa (fra 0.4 e 2 m) non c'era un solo punto nelle mappe delle 8 run
precedenti; la run `1055`, con i soli alberi gia' corretti, le e' passata a 0.31 m. Con la
correzione completa la facciata e' in mappa al 100% in tutte e 3 le run.
**Correzione**: `make_parco.py` (funzione `geometrie_per_lidar`) aggiunge a ogni modello del parco
un visual `per_lidar`, cioe' la sua stessa mesh di collisione con ogni triangolo nei due versi
(`meshes/*_lidar.dae`), trasparente e senza ombre. Il lidar la vede, perche' ignora la trasparenza
del MATERIALE; telecamere e GUI no: immagini identiche pixel per pixel prima e dopo, da 4 punti di
vista. Tre dettagli misurati per arrivarci:
- in formato OBJ, o in COLLADA senza normali, Gazebo ignora colore e trasparenza del materiale e la
  mesh esce bianca e opaca;
- con le ombre accese la copia trasparente proietta comunque un'ombra piena;
- lo stop, una lastra di 2 cm, e' visto al 99% sia da davanti sia da dietro; di taglio i raggi
  radenti lo mancano (29%), come succederebbe anche a un lidar vero.

Collisione, aspetto, `parco.sdf` e `parco_ostacoli.npz` sono invariati. Il blocco nei `model.sdf`
si rigenera con `python3 scripts/make_parco.py`.

Verifica in volo (comando qui sopra; per la RAM disponibile, 3.6 GB con l'interfaccia aperta, si
e' usato `PF_MIN_RAM_GB=3`, che a drone singolo basta):

| run | correzione | esito | esplorazione | missione | margine 3D | errore stima mediano |
|---|---|---|---|---|---|---|
| parco `1055` | solo alberi | finita | 221 s | 263 s | **0.31 m** (facciata invisibile) | 0.07 m |
| parco `1103` | solo alberi | finita | 204 s | 255 s | 0.43 m | 0.09 m |
| parco `1111` | solo alberi | finita | 164 s | 199 s | 0.40 m | 0.09 m |
| parco `1136` | completa | finita (2 frontiere irraggiungibili) | 246 s | 278 s | 0.44 m | 0.06 m |
| parco `1144` | completa | finita | 245 s | 293 s | 0.63 m | 0.08 m |
| parco `1153` | completa | finita | 171 s | 215 s | 0.49 m | 0.08 m |

Nelle mappe del planner la superficie di collisione dei rami fra 0.5 e 1.6 m e' coperta al 76-79%,
contro il 48-52% delle run precedenti: querce 87-100%, pini 69-81%. Il resto e' l'interno delle
chiome dei pini, nascosto dai rami esterni. Un lidar vero vede il fogliame fitto in modo simile.

La pianta 2D degli ostacoli non basta all'aperto: proietta nella fascia 0.4-1.9 m il tetto delle
auto e le chiome, sopra e sotto cui il drone passa davvero (run `0054`: 0.00 m in 2D, 0.53 m in 3D).
`make_parco.py` salva ora anche gli ostacoli 3D in `parco_ostacoli.npz` e `analisi_campagna.py`
li usa per il margine quando ci sono (la copertura delle pareti, che si calcola dai box, per il
parco resta `-`). La build di questa cartella e' stata rifatta qui: quella copiata puntava ancora
a `Drone_20261013`.

**Quota minima in crociera (18 ottobre 2026).** Nella run `1248` il drone e' sceso a **0.13 m**
(colonna `q.min` di `analisi_campagna.py`, quota vera con il corpo appoggiato = 0), con la stima
giusta (errore sotto 4 cm). Nelle run precedenti del parco il minimo stava fra 0.38 e 0.66 m. Per il
drone vero, col Jetson a bordo, e' rischio di urto col terreno. Le cause sono due, misurate sui
registri di volo (`trajectory_setpoint` contro la verita' del simulatore) e sui registri
dell'esplorazione.

1. **Riferimento fermo con anticipo verso il basso** (`1248`, all'aperto, nessun ostacolo entro
   3 m). Dopo un cambio di piano il drone e' rimasto piu' di 1 m indietro (finiva una frenata in
   controsenso) e il piano nuovo partiva con accelerazione verticale -1.27 m/s2. `traj_server`
   rallenta l'orologio della traiettoria con lo scarto e lo ferma a 1 m (`track_err_stop`), ma
   pubblicava velocita' e accelerazione PIENE dell'istante congelato: per 2 s la posizione stava
   ferma con vz -0.46 m/s e az -1.05 m/s2. Il limite di scostamento del ponte (`max_setpoint_jump`,
   0.65 m in 3D) teneva il riferimento a 0.65 m dal drone, quindi lo trascinava giu' con lui: da
   0.93 a 0.61 m (quota PX4). E con `MPC_Z_P` = 1 PX4 si assesta dove la correzione di posizione
   pareggia l'anticipo, 0.47-0.51 m sotto il riferimento: 0.13 m. In orizzontale l'anticipo
   congelato (ay -1.65) annullava la correzione verso il riferimento, quindi lo scarto non calava e
   l'orologio restava fermo, fino al piano successivo (2.4 s dopo). Il pavimento del ponte
   (`min_flight_z`, 1.0 m mondo = 0.77 m quota PX4) valeva solo per la posizione del bersaglio.
2. **Percorso sotto ostacoli sovrastanti** (`0054`, `1103`, `1153`, `1320`: 0.38-0.43 m). Quasi
   tutti i tratti in cui il riferimento e' sceso sotto il pavimento hanno l'ostacolo piu' vicino
   0.6-0.7 m SOPRA il drone: la piattaforma dell'area giochi (fondo a ~1.34 m) e le chiome delle
   querce. Il pavimento lo conosceva solo il ponte: l'A* passava li' sotto, e l'ottimizzatore
   (`optimization.dist0` 0.7 m dal terreno E dalla piattaforma) metteva la traiettoria a meta',
   0.62-0.69 m. Il filtro di sicurezza del ponte allontanava poi il riferimento dalla piattaforma
   togliendogli la componente verso l'alto, cioe' spingendolo in basso (fino a 0.75 m mondo), e
   aggiungeva anticipo verso il basso.

La frequenza dell'anticipo congelato con orologio fermo, cercata in 33 run (campioni consecutivi con
velocita' e accelerazione identiche e non nulle): un solo caso netto, la `1248`. Raro, ma l'unico
che porta il drone a terra.

**Correzioni** (commentate nel codice con questi numeri):

- `traj_server.cpp`: quando l'orologio rallenta oltre la meta' (scarto oltre 0.65 m) l'anticipo
  pubblicato si sfuma fino a zero a orologio fermo: velocita' x g, accelerazione x g^2, g = scala/0.5.
  Fino a 0.65 m di scarto resta pieno come prima. PROVATO E SCARTATO (run `1401`, `1427`): le
  derivate esatte del riferimento rallentato, `v*scala` e `a*scala^2`, gia' da 0.3 m di scarto.
  Toglievano la spinta di recupero al drone rimasto indietro: nella `1427` scarto orizzontale
  riferimento-drone mediana 0.22 m e oltre 0.3 m nel 24% del volo, contro 0.07-0.13 m e 2-11% delle
  8 run precedenti; con lo sfumare le run finali sono a 0.10-0.16 m e 8-15%.
- `position_command_bridge.cpp`:
  - il limite di scostamento non abbassa il riferimento sotto `min(rampa, min_flight_z)`;
  - niente anticipo verso il basso (vz, az) quando il riferimento e' entro 5 cm dal pavimento o il
    drone e' gia' sotto (`anticipoAlPavimento`);
  - `min_flight_z_hard` (0.8 m mondo): sotto questa quota filtro di sicurezza e limite di frenata non
    spingono piu' il riferimento. Lascia 0.54 m dalla piattaforma (safety_dist 0.5);
  - messaggi `[Quota]` nel registro dell'esplorazione, con un riepilogo all'atterraggio: per quanti
    secondi il piano e' sceso sotto il pavimento, quante volte l'anticipo e' stato tolto e il
    riferimento trattenuto.
- Pianificatore: il pavimento entra anche nei percorsi. `astar.quota_minima` (A* a griglia e
  cinodinamico): mai sotto `min(pavimento, quota di partenza, quota di arrivo)`, cosi' le ricerche che
  partono piu' in basso (decollo a 0.85 m) restano possibili; il cinodinamico ha 10 cm di tolleranza
  sulla prima espansione, altrimenti un drone al pavimento che scende non trova primitive. Sotto la
  piattaforma la fascia gonfiata (0.3 m) scende a ~1.0 m e il percorso gira intorno o sopra.
  Nell'ottimizzatore (`bspline_optimizer.cpp`) il pavimento e' un ostacolo virtuale nel costo di
  distanza, peso `optimization.peso_quota` 5: sotto una chioma a 1.42 m l'equilibrio passa da 0.72 a
  ~0.95 m. Con la sola correzione dell'A* (run `1401`) il piano scendeva ancora sotto 1.0 m per
  9.3 s, fino a 0.69 m; con anche l'ottimizzatore (`1427`) per 0 s.
- Il launch usa un solo valore (`quota_minima` = 1.0 m mondo) per ponte e pianificatore;
  `PAVIMENTO_PERCORSI=0` nell'ambiente lo toglie ai percorsi, per i confronti.

Verifica (Jetson, `ODOM_SOURCE=lio VEL_MAX=1.0 VISITA=0`, `PF_MIN_RAM_GB=3`):

| run | versione | esito | missione | q.min | margine 3D | scarto xy mediana | piano sotto il pavimento |
|---|---|---|---|---|---|---|---|
| parco `1401` | senza ottimizzatore, derivate esatte | finita, atterrata | 325 s (*) | 0.67 m | - | 0.12 m | 9.3 s, fino a 0.69 m |
| parco `1427` | derivate esatte | finita, atterrata | 253 s | 0.68 m | - | 0.22 m | 0 s |
| parco `1644` | **finale** | finita, atterrata | 274 s | **0.69 m** | 0.46 m | 0.10 m | 0 s |
| parco `1652` | **finale** | finita, atterrata | 249 s | **0.74 m** | 0.42 m | 0.15 m | 0 s |
| parco `1701` | **finale** | finita, atterrata | 269 s | **0.66 m** | 0.40 m | 0.10 m | 0 s |
| ufficio `1709` | **finale** | finita, atterrata | 332 s | **0.77 m** | 0.50 m | 0.16 m | 0 s |
| ufficio `1721` | **finale** | finita, atterrata | 377 s | **0.75 m** | 0.40 m | 0.15 m | 1.6 s, fino a 0.83 m |

Colonne q.min, margine e missione da `analisi_campagna.py`; tutte e 5 le run finali sono "pulite"
(nessun contatto, inclinazione massima 15-20 gradi, copertura 3D dell'ufficio 99.9-100%).
Prima, nel parco (`1055`-`1320`, 8 run): missione 199-293 s (mediana ~259 s), q.min 0.13-0.66 m,
margine 0.31-0.63 m. In cubicle_office (`1907`-`1236`, 14 run): missione 283-378 s (mediana ~305 s;
`1236`, l'ultima prima di queste correzioni, 376 s), q.min 0.59-0.72 m. Quindi la quota minima sale
a 0.66-0.77 m in entrambi i mondi; i tempi restano nella forbice di prima (parco mediana 269 s contro
~259, ufficio 332 e 377 s contro 376 della `1236`), con due run per l'ufficio troppo poche per dire
di piu'. Da tenere d'occhio: in ufficio 1 "Plan fail" per run (prima 0), dalla ricerca cinodinamica
fallita due volte fra 1.28-1.38 m e 1.24 m di quota, cioe' sopra il pavimento; puo' essere il nuovo
pavimento come no. Le ripianificazioni "drone indietro" (18 per run in ufficio) erano gia' 13 nella
`1236`, contro 0-3 la sera del 17: l'aumento viene da prima di questo lavoro.
(*) Nella `1401` la velocita' in moto e' la stessa delle altre (1.01 m/s); il tempo in piu' viene
da un percorso piu' lungo (228 m contro 160-222) e da 76 s di attesa nella fase finale, gia' vista
prima delle correzioni nelle `1136` e `1248` (71 e 69 s).

---

## Come si lancia

Non serve caricare ROS a mano: lo fanno gli script.

**Senza GPS**, posizione da FAST-LIO (lidar + IMU), su un mondo QUALSIASI:

```bash
cd ros2_ws && ODOM_SOURCE=lio ./scripts/run_multi_exploration.sh cubicle_office 1
cd ros2_ws && ODOM_SOURCE=lio ./scripts/run_multi_exploration.sh walls 1
cd ros2_ws && ODOM_SOURCE=lio ./scripts/run_multi_exploration.sh /percorso/del/mio_mondo.sdf 1
```

Il primo argomento e' il mondo: il nome di uno di quelli del progetto
(`ros2_ws/src/c2x_worlds/worlds/`: `cubicle_office`, `office_small`, `parco`,
`open_field`, `empty_room`) o di PX4 (`PX4-Autopilot/Tools/simulation/gz/worlds/`),
oppure il percorso di un file `.sdf`. Il secondo e' il numero di droni. Togliendo
`ODOM_SOURCE=lio` si vola col GPS simulato (configurazione storica).

Il mondo lo prepara `scripts/prepara_mondo.py` (dettagli sotto, "Missioni su
qualsiasi mondo"): per quelli gia' tarati usa partenze e configurazione misurate
nelle campagne, per gli altri ricava dalla geometria la zona da esplorare, un punto
di partenza libero e, se il mondo e' aperto, un recinto. Si possono scegliere a mano
con `SPAWN="x,y[,yaw];..."` (una posa per drone) e `BOX="xmin,ymin,xmax,ymax"`.

Cosa succede in una run: decollo a 1 m, giro di 360 gradi sul posto (~13 s),
esplorazione, rientro sopra il punto di decollo, atterraggio verticale li' (entro
pochi centimetri), mappa 3D ricostruita e **aperta in una finestra** che si ruota
col mouse (C = soffitto, Q = chiudi; resta aperta anche dopo il Ctrl+C).

Opzioni (da mettere davanti al comando):

| variabile | default | effetto |
|---|---|---|
| `ODOM_SOURCE` | `px4` | `lio` = senza GPS (FAST-LIO); `livo` = senza GPS con FAST-LIVO2 (lidar + IMU + telecamera, prova: vedi "FAST-LIVO2") |
| `VEL_MAX` | `1.0` | velocita' massima del pianificatore, m/s. office_small a 1.0: pulito (V19). **cubicle_office: usare 0.7** (vedi Stato del lavoro) |
| `GIRO_INIZIALE` | `0.5` | velocita' del giro iniziale in rad/s, `0` = niente giro |
| `MAPPA_REALISTICA` | `1` | `0` = niente mappa 3D |
| `FOTO` | `1` | telecamera frontale accesa SOLO per colorare la mappa 3D (fotogrammi chiave temporanei in `/tmp`, non archiviati); `0` = mappa 3D grigia, telecamera mai resa |
| `MOSTRA_MAPPA` | `1` | `0` = non aprire la finestra della mappa 3D |
| `ARCHIVIA` | `1` | `0` = non copiare la run in `risultati_prova/` |
| `SPAWN` | calcolata | partenze `"x,y[,yaw];..."`, una per drone |
| `BOX` | calcolata | zona da esplorare `"xmin,ymin,xmax,ymax"` |
| `RECINTO` | `auto` | `1`/`0` = aggiungere o no il recinto attorno alla zona (di serie solo nei mondi aperti) |

**Ogni run viene archiviata da sola** a fine run (e anche se la si chiude prima
con Ctrl+C, con il suffisso `_interrotta`) in
`risultati_prova/<data>_<ora>_<mondo>_<odom>_v<VEL_MAX>/`, nel formato `rel_*`
delle campagne, **uguale per ogni mondo**: il risultato della missione e' la **mappa
3D** `rel_mappa3d_1_<d>/` (`mappa_realistica.ply`, la superficie colorata dalla
telecamera, con `anteprima_alto.png` e `anteprima_prospettiva.png`), aperta anche in
una finestra a fine run. Con lei: `run.txt` (impostazioni, mondo ed esito:
esplorazione finita, crash, divergenze di FAST-LIO), `rel_mondo.json` (mondo di
origine, partenze, zona, recinto), `rel_mondo.sdf` e `rel_config.yaml` (il mondo e la
configurazione con cui si e' volato), `rel_run_1.log` (uscita del terminale),
`rel_expl_1.log`, `rel_px4_1_<i>.log`, `rel_ulog_1_<i>.ulg` (registro di volo) e
`rel_map_1.pcd` (mappa del pianificatore, per le analisi). L'immagine dall'alto con
la percentuale di pareti viste (`rel_mappa_1.png`, `disegna_mappa.py`) non si produce
piu' (18 ottobre 2026): funzionava solo nei mondi fatti di box.
Non si copiano `punti.npz` (100-400 MB, serve solo a rifare la ricostruzione 3D) ne' i
fotogrammi della telecamera, che servono solo a colorare la mappa 3D (dal 17 ottobre
2026): entrambi restano in `/tmp/c2x_run_multi/mappa_realistica_drone<N>/` fino alla
run successiva. Peso: ~300 MB per una run su cubicle_office (quasi tutto il
`.ulg`), ~80 MB su office_small. Le campagne (`reliability_campaign.sh`) non
passano di qui: archiviano in `/tmp/c2x_reliability`.

Una mappa 3D gia' salvata si riapre con:

```bash
python3 ros2_ws/scripts/mostra_mappa.py risultati_prova/V13_LIO/rel_mappa3d_3_1
```

### Missioni su qualsiasi mondo (18 ottobre 2026)

`prepara_mondo.py <mondo> --droni N --uscita <cartella>` (lo chiama lo script di run,
in `/tmp/c2x_run_multi/mondo/`) scrive il mondo normalizzato, la configurazione dello
stack e le partenze:

- **mondo normalizzato**: nome uguale al file, plugin del mondo tolti (li carica il
  `server.config` di PX4, lo stesso per tutti), fisica a 250 Hz come i mondi del
  progetto, coordinate geografiche e luce se mancano, URI relativi resi assoluti;
- **mondi tarati** (`cubicle_office`, `office_small`, `empty_room`, `open_field`,
  `parco`): partenze e configurazione delle campagne, come prima;
- **ogni altro mondo**: `geometria_mondo.py` legge la geometria di COLLISIONE (modelli
  scritti nel mondo e inclusi con `model://`, URL di Fuel gia' scaricati, percorsi
  relativi; box, cilindri, sfere, capsule, ellissoidi, piani, mesh `.dae`/`.obj`/`.stl`).
  Da li':
  - zona da esplorare = ingombro degli ostacoli fra 0.1 e 2.5 m, +0.5 m, al massimo
    40 x 40 m;
  - quota della zona 2.5 m con un soffitto, 2.2 m senza;
  - partenza = punto libero (>= 1.5 m dagli ostacoli fra 0.3 e 1.3 m, se non c'e'
    1.2, 1.0, 0.8 m; nulla sopra sotto 1.9 m) piu' vicino al centro dello spazio
    libero, gli altri droni a 3 m;
  - configurazione = quella di `cubicle_office` (con tutte le tarature e le
    protezioni) con zona, mappa e celle di spartizione adattate;
  - la mappa del pianificatore non e' piu' per forza centrata nell'origine
    (`sdf_map.map_origin_x/y`, nuovo in `sdf_map.cpp`).
- **mondi aperti**: se piu' del 5% dei raggi orizzontali da punti liberi non incontra
  nulla, si aggiunge un recinto alto 4 m attorno alla zona (`RECINTO=0/1` per
  forzare). Senza, i raggi del lidar che non tornano non liberano spazio e
  l'esplorazione non si chiude mai: e' il motivo per cui `open_field` e il parco
  hanno recinzione e siepe.

Limiti: il terreno deve essere a quota 0 (lo stack vola a quote assolute: altrimenti
si esce con un errore); heightmap e mesh in altri formati non vengono lette
(avviso); i modelli presi da Fuel devono essere gia' in `~/.gz/fuel` perche' la loro
geometria entri nel calcolo (Gazebo li scarica al primo avvio, quindi basta una prima
run o `gz fuel download`). `analisi_campagna.py <run>` riconosce da solo il mondo
archiviato e, per quelli non tarati, misura il margine dalla geometria 3D.

| run (18 ottobre) | mondo | esito | esplorazione | missione | margine 3D |
|---|---|---|---|---|---|
| `1226_walls` | `walls.sdf` di PX4, **mai visto**: aperto, recinto aggiunto | finita, atterrata | 145 s | 186 s | 0.40 m |
| `1236_cubicle_office` | tarato | finita, atterrata | 347 s | 376 s | 0.71 m |
| `1248_parco` | tarato | finita, atterrata (2 frontiere irraggiungibili) | 250 s | 277 s | 0.52 m (*) |
| `1256_walls` e `1310_walls` | `walls.sdf`, dalla COPIA del progetto | finite, atterrate | 137 / 158 s | 181 / 197 s | 0.47 / 0.68 m |

(*) In quella run il drone e' sceso a 0.13 m di quota (stima giusta, riferimento a
0.61 m con velocita' comandata -0.46 m/s verso il basso): difetto di inseguimento in
discesa, non legato a queste modifiche, da approfondire.

### Progetto trasportabile (18 ottobre 2026)

Gli script e i sorgenti usavano gia' percorsi relativi; le BUILD no. Provato
copiando il progetto in un'altra cartella (`rsync -a`, senza `risultati_prova/` e
senza registri di volo) e volando da li': durante il volo si sono letti i file aperti
da ogni processo (`/proc/<pid>/maps` e `fd`) e alla fine si e' controllato che
nella cartella originale non fosse stato scritto nulla. Tre perdite trovate e chiuse:

- `ros2_ws/install` (colcon `--symlink-install`) era fatto di 4923 collegamenti
  ASSOLUTI verso `build/` e `src/`: una copia avrebbe usato i file dell'originale.
  `scripts/percorsi_relativi.py` li rende relativi e lo richiama il controllo prima
  di ogni run, perche' una build li ricrea assoluti;
- `PX4-Autopilot/build/.../rootfs/gz_env.sh` (percorsi di modelli, mondi e plugin di
  Gazebo) lo scrive la build con percorsi assoluti: ora lo riscrive lo script di run
  a ogni avvio; e i plugin Gazebo di PX4 cercavano `libOpticalFlow.so` nella build
  in cui sono stati compilati (che qui e' `Drone_20261013`!): lo script di run
  imposta `LD_LIBRARY_PATH` sulla build di questa copia;
- FAST-LIO scriveva i suoi file di debug nella cartella dei sorgenti fissata in
  compilazione (`ROOT_DIR`): ora in `/tmp/c2x_fast_lio/` (o `$C2X_LIO_DIR`).

Dopo le correzioni: dalla copia nessun file aperto dall'originale ne' da
`Drone_20261013`, niente scritto nell'originale, missione completa. Lo script vecchio
a drone singolo `run_exploration.sh` ora richiama `run_multi_exploration.sh` con un
drone (l'originale e' in `scripts/archivio/`), cosi' ogni missione passa dallo
stesso percorso e da' gli stessi risultati.

**Per verificare che una run sia volata davvero senza GPS:**

```bash
cd ros2_ws && ./scripts/verifica_senza_gps.sh
```

Legge il registro di volo e controlla cinque condizioni indipendenti. Serve perché
il GPS simulato continua a pubblicare anche quando EKF2 lo ignora: contare i
campioni non dice nulla, bisogna guardare se EKF2 ha creato una sorgente GNSS.

**Campagna di prove ripetute + pagella:**

```bash
cd ros2_ws && ODOM_SOURCE=px4 ./scripts/reliability_campaign.sh 5 PROVA 900 office_small 1
python3 scripts/analisi_campagna.py        # una riga per run + riepilogo
```

`analisi_blocchi.py <exploration.log>...` trova gli intervalli in cui il drone resta
fermo mentre dovrebbe esplorare e ne indica la causa dominante.

`analisi_campagna.py` misura per ogni run: esito, frontiere rimaste, volume
esplorato, rientro e atterraggio, margine minimo VERO dagli ostacoli (geometria del
mondo contro la posizione vera del simulatore), quota minima, inclinazione e
velocita' massime, errore della stima, e per la mappa del planner (`cop%`,
`prec%`) e per la mappa 3D (`cop3d`, `pre3d`):
- **copertura** = quota delle superfici di parete **visibili dalla zona in cui il
  drone puo' volare** che compaiono nella mappa, dal lato giusto della parete;
- **precisione** = punti di parete entro 15 cm da una parete vera;
- `ev>0.5s` / `ev max`: buchi della stima di FAST-LIO verso PX4 (senza GPS);
- `pav` / `pend`: quota (vero 0) e pendenza del pavimento della mappa 3D, cioe'
  l'errore verticale che la precisione non vede.

Una run conta come **pulita** solo se e' conclusa, atterrata, senza contatti con
gli ostacoli, senza inclinazioni da urto e con copertura finale (mappa 3D) almeno
del 95%.

Fino a V12 la copertura contava anche superfici che nessun volo puo' vedere
(interno delle stanze chiuse, fessure da 30 cm, retro dei muri) ed escludeva le
pareti perimetrali: il suo tetto fisico era ~73%, per questo sembrava ferma al
70%. Le stesse run V12 senza GPS, misurate sulle superfici visibili, erano al
90-98%.

---

## Robustezza del volo a drone singolo (ottobre 2026)

Analisi e correzioni fatte su `office_small` con un drone, misurate con campagne di
run ripetute (`risultati_prova/`, una cartella per campagna con `pagella.txt`).
Ogni modifica e' commentata nel codice con la misura che la motiva.

**Guasti trovati nelle campagne di riferimento (codice originale):**
- tre urti contro le pareti su otto run: il pianificatore considerava libero un
  punto a 20 cm da un muro (gonfiaggio 0.199 m) con un velivolo largo 0.35 m di
  raggio, e accettava punti di vista attaccati alle pareti;
- traiettorie oltre i limiti dinamici: anticipo di velocita' a 1.3 m/s con
  `max_vel` 0.6, perche' l'ottimizzatore tratta i limiti come penalita';
- la fermata reattiva su lidar si accendeva e spegneva piu' volte al secondo
  vicino ai muri (riferimento a onda quadra) - causa diretta di un urto - e dopo
  bloccava il drone in cicli "ferma 5 s, rilascia, ferma";
- PX4 con i limiti di serie da esterno (12 m/s, 45 gradi) e senza tetto di quota
  nel ponte: dopo una divergenza della stima il riferimento e' salito a 5.8 m;
- decollo dichiarato concluso a 0.5 m di quota;
- frontiere irraggiungibili mai archiviate (727 pianificazioni fallite in una run);
- urti non rilevati (inclinazione a 76 gradi contata come run riuscita), nessun
  atterraggio dopo un crash, riarmo automatico in volo, drone in hovering infinito
  se un nodo moriva.

**Correzioni principali:**

| dove | cosa |
|---|---|
| `position_command_bridge.cpp` | filtro di sicurezza continuo al posto della fermata a scatti: toglie solo la componente del moto verso l'ostacolo, guardando sia il riferimento sia il velivolo; tetto di quota; decollo concluso entro 15 cm; niente riarmo dopo il decollo |
| `planner_manager.cpp` | ogni traiettoria viene rallentata uniformemente se supera i limiti di velocita'/accelerazione (il percorso non cambia) |
| `frontier_finder.cpp` | punti di vista ad almeno 0.55 m dagli ostacoli e nella fascia di quota di volo; **ricontrollati a ogni aggiornamento della mappa** (un punto di vista finito a ridosso di un muro scoperto dopo bloccava il drone in "No path to next viewpoint" fino a 616 s); rimozione delle frontiere che tiene allineata la matrice dei costi; diagnostica dei motivi di scarto (`[PuntiDiVista]` nel log); anello di campionamento 0.5-2.0 m (copertura delle pareti da ~70% a ~82%); **lista nera** dei punti di vista per cui la ricerca di percorso fallisce davvero, cosi' una sacca visibile ma senza accesso non blocca il drone |
| `cloud_transform_bridge.cpp` | **ogni nuvola posata con la posa del suo istante**: senza GPS la posa di FAST-LIO della stessa scansione (o interpolata fra le due vicine), col GPS la posa di PX4 interpolata all'istante della scansione (la nuvola arriva ~30 ms dopo). Prima si usava l'ultima posa ricevuta: strisciate a ventaglio e pareti ispessite verso l'esterno |
| airframe 22000 (bussola) | declinazione magnetica nulla in simulazione: il campo di Gazebo non ha declinazione mentre EKF2 applicava quella di Zurigo, e l'imbardata stimata era sbagliata di 3.5 gradi costanti (pareti lontane doppie). **Su un drone vero rimettere `EKF2_DECL_TYPE 3`** |
| `c2_exploration_manager.cpp` (centro cella) | se dal centro della cella assegnata non si raggiunge nessuna frontiera si riprova dalla posizione del drone (blocchi di 354-616 s) |
| `disegna_mappa.py` | immagine di fine run: solo le pareti, con sopra la pianta vera e i punti colorati per errore (prima si disegnavano anche pavimento e soffitto: una macchia piena) |
| `astar2.cpp` | ricerca "paziente": se scade il tetto di 10 ms si riprova con 0.15 s prima di dichiarare un punto irraggiungibile |
| `c2_exploration_fsm.cpp` | crash rilevato a 50 gradi per 0.2 s con atterraggio; stato finale non reversibile; tempo massimo di esplorazione; archiviazione delle frontiere irraggiungibili dopo 30 fallimenti di fila; atterraggio anche se il drone finisce vicino alla partenza |
| `lio_to_px4_bridge.cpp` | rilevatore di divergenza di FAST-LIO: smette di mandare la stima a PX4 e fa atterrare |
| airframe 22000 | limiti da ambiente chiuso (1.2 m/s, 30 gradi, 1 m/s in salita), perdita offboard -> atterraggio, ritorno a casa a 1.2 m |
| launch | se un nodo critico muore si spegne tutto e PX4 atterra da solo |
| `multi_drone_office_small.yaml` | gonfiaggio 0.3 m, vincoli dei punti di vista, tempo massimo 600 s |
| `multi_drone_cubicle_office.yaml` | stesse correzioni di office_small (gonfiaggio 0.3, punti di vista a 0.55 m dagli ostacoli e a 1.0-1.7 m di quota, accelerazione 2.0, frontiere irraggiungibili archiviate), tempo massimo 1200 s per l'ufficio 31x31 m. Validate a 1.0 m/s senza GPS nelle campagne V29-V30 (5/5 run pulite) |
| `fastdds_profilo.xml` | **memoria condivisa da 16 MB per DDS**. Una scansione lidar (~650 KB) superava il segmento di serie (512 KB), passava su UDP a frammenti e FAST-LIO (in "best effort") ne perdeva a raffica: 14-35 buchi oltre 0.5 s per run nella stima verso PX4, massimo 1.5-3.0 s. Oltre ~1 s PX4 scendeva alla cieca (un drone e' arrivato a 21 cm da terra) e un buco di 2.3 s ha preceduto un ribaltamento. Con il profilo: 0 buchi, massimo 0.20 s, FAST-LIO a 10 Hz costanti |
| `run_multi_exploration.sh` (quota di partenza) | il drone nasce a 0.227 m, l'altezza vera del corpo appoggiato sui pattini, e la stessa quota va ai ponti come origine. Con 0.1 tutta la mappa (anche col GPS) era 12.7 cm piu' bassa del vero e il drone volava 13 cm piu' alto di quanto credesse |
| `position_command_bridge.cpp` (atterraggio) | a fine missione il pianificatore riporta il drone entro 1.5 m dalla partenza, poi il ponte fa l'ultimo tratto in linea retta e lento (0.4 m/s, filtro di sicurezza attivo) fino alla verticale esatta del decollo e atterra li'. Prima si atterrava a 1.2-1.5 m dalla partenza; ora a 2 cm. Le emergenze (crash, tempo scaduto) atterrano ancora sul posto |
| `planner_manager.cpp` (limiti dinamici) | il controllo dei limiti misura i picchi sulla curva vera (campionata ogni 20 ms) e allunga solo la parte centrale della traiettoria (`lengthenTime`), lasciando invariati stato iniziale e finale. Prima riscalava tutti i nodi, quindi la traiettoria nuova partiva piu' lenta del drone: a ogni ripianificazione (~1.5 s) una frenata del 30%, cioe' il volo a strappi |
| `bspline_optimizer.cpp` | limiti di velocita'/accelerazione sul MODULO invece che asse per asse (`optimization.feasibility_norm`): in diagonale l'ottimizzatore arrivava a 1.4-1.7 volte il limite e il controllo a valle rallentava 471 traiettorie su 472 |
| `c2_exploration_fsm.cpp` (rientro) | durante il rientro la ripianificazione periodica prosegue verso casa invece di passare da IDLE (che troncava la traiettoria in corso): rientro in una tappa invece di 12 |
| `fast_lio/config/c2x_sim.yaml` | mappa interna di FAST-LIO a 0.2 m invece di 0.5. Misurato rifacendo girare FAST-LIO sui dati registrati di un volo contro la posa vera di Gazebo: errore d'inclinazione da 0.69 a 0.13 gradi, errore di quota da 9 a 3.6 cm. Conta la risoluzione della mappa, non della scansione |
| `position_command_bridge.cpp` (giro) | giro di 360 gradi a quota di decollo prima del via; l'FSM si dichiara pronta solo dopo (`volo_pronto`). Il decollo non ruota piu' il drone verso nord |

---

## Mappa realistica 3D (ottobre 2026)

Richiesta di partenza: usare [Glob3R](https://github.com/aigc3d/Glob3R). Ad
ottobre 2026 quel repository contiene solo il README: niente codice, pesi o
licenza. Inoltre Glob3R ricostruisce una scena da sole foto (struttura dal
movimento), mentre qui c'e' gia' un lidar con le pose: la geometria del lidar e'
piu' precisa di qualunque ricostruzione da immagini. Si e' quindi fatta la
stessa cosa nel modo piu' diretto: **geometria dal lidar, colore dalla
telecamera**. Se Glob3R rilascera' il codice si potra' usare per la sola parte
del colore/dettaglio fine, al posto di `colora()`.

Due script, lanciati da `run_multi_exploration.sh` (disattivabili con
`MAPPA_REALISTICA=0`):

| script | quando | cosa fa |
|---|---|---|
| `registra_mappa.py` | dal decollo (parte prima dello stack, cosi' registra anche il giro iniziale) | accumula le nuvole gia' posate (`/drone_N/map_ros/cloud`) in voxel da 2 cm con la direzione da cui ogni punto e' stato visto; salva un fotogramma della telecamera frontale ogni 0.3 m o 10 gradi, con la posa della telecamera all'istante della foto |
| `ricostruisci_mappa.py` | a fine run (~15 s) | superficie di Poisson (profondita' 9, normali su 15 cm rivolte verso lo spazio libero), taglio delle parti inventate a piu' di 10 cm da un punto misurato, levigatura di Taubin, decimazione a 1.5 M triangoli. Niente taglio per densita': toglieva proprio le pareti viste da lontano (copertura 93.6% -> 98.1% senza, precisione invariata). Si tengono anche le celle da 2 cm colpite una sola volta (`--min_conteggio 1`): sono le parti alte delle pareti viste di sbieco, e con FAST-LIO corretto non portano rumore (copertura 98.7-99.7% -> 99.97-100%); ogni vertice prende il colore dei fotogrammi da cui e' davvero visibile (controllo di occlusione col raggio telecamera-vertice) |

Uscite in `/tmp/c2x_run_multi/mappa_realistica_drone1/`:
`mappa_realistica.ply` (superficie colorata, ~45 MB, si apre con MeshLab o
CloudCompare), `nuvola_colorata.ply`, `anteprima_alto.png`,
`anteprima_prospettiva.png`. Le campagne le copiano in `rel_mappa3d_<run>_1/`.

Per rifarla a mano su una cartella gia' registrata (serve Open3D 0.20,
installato con `pip install --user open3d`):

```bash
/usr/bin/python3 ros2_ws/scripts/ricostruisci_mappa.py /tmp/c2x_run_multi/mappa_realistica_drone1
```

Nota: in `office_small` le pareti diagonali sono fatte di blocchetti da 20-80 cm,
quindi i "gradini" che si vedono nella mappa 3D sono veri, non un difetto.

---

## FAST-LIVO2 al posto di FAST-LIO (prova, 18-19 ottobre 2026)

FAST-LIVO2 (https://github.com/hku-mars/FAST-LIVO2) stima la posa da lidar, IMU **e
telecamera**: sui dati veri del paper e' piu' preciso di FAST-LIO2 (errore medio 4.5 contro
15 cm). Qui gira come alternativa a FAST-LIO, scelta all'avvio; tutto il resto (ponte verso
PX4, controllo di salute, pianificazione, mappa 3D) e' lo stesso.

```bash
cd ros2_ws && PF_MIN_RAM_GB=3 ODOM_SOURCE=livo ./scripts/run_multi_exploration.sh parco 1
```

Servono `MAPPA_REALISTICA=1` e `FOTO=1` (i valori di serie): la telecamera arriva in ROS
dal ponte che avvia `registra_mappa.py`.

**Licenza: GPLv2**, come FAST-LIO. Non si puo' togliere modificando il codice: per un uso
commerciale serve la licenza alternativa offerta dagli autori o una riscrittura da zero.

**Dove sta e cosa e' cambiato.** `ros2_ws/src/fast_livo2/` (con `PROVENIENZA.md`): il codice
originale (commit 0d2c034) con le 5 patch ROS 2 di Intel (Robotics AI Suite) e le modifiche
per questo drone. Dipendenze in `ros2_ws/src/terze_parti/`: Sophus 1.22.10, rpg_vikit per
ROS 2 e il driver Livox (dal 19 ottobre quello completo, vedi "Collaudo del drone vero"). Le copie
scaricate (repository di FAST-LIVO2 e patch Intel) sono state tolte il 19 ottobre: i sorgenti usati
sono nello spazio di lavoro, l'origine di ognuno in `PROVENIENZA.md`. Modifiche principali:

| dove | cosa |
|---|---|
| `config/c2x_sim.yaml`, `config/camera_c2x_sim.yaml` | sensori del drone simulato: la nuvola di `lidar_lio_adapter` (formato Velodyne, solo punti finiti), l'IMU del lidar, la telecamera frontale 640x480 a 90 gradi (fx = fy = 320) con la posa dal modello; LIO a 3 iterazioni come FAST-LIO; `uav.gravity_align_en: false` (vedi sotto) |
| `LIVMapper.cpp`, ciclo principale | un esecutore solo e `spin_all` invece di `spin_some`: prima a ogni giro si prendeva UN messaggio per topic, l'IMU a 200 Hz restava in coda e il ritardo della stima cresceva fino a 0.9-1.4 s, poi la divergenza (3 crash) |
| `LIVMapper.cpp`, ritardo limitato | a ogni ciclo si riparte dall'immagine piu' recente gia' coperta da lidar e IMU (le piu' vecchie si scartano); coda delle immagini di 2 invece di 200000 |
| `LIVMapper.cpp`, uscite | iscrizione alla nuvola in "best effort" (con "reliable" DDS non consegnava nessuna scansione); odometria con l'istante della misura, pubblicata dopo l'aggiornamento visivo; `/lio_salute` come FAST-LIO (atterraggio su stima degradata; agganciati in volo 2300-5300, soglia 500); nuvola colorata, immagine, percorso solo con qualcuno in ascolto |
| `main.cpp` | stdout in `/dev/null` (`C2X_LIVO_STDOUT=1` per la diagnosi): ~600 righe al secondo nella pipe di `ros2 launch` (10 MB in 4 minuti) |
| `CMakeLists.txt` | tempi per fotogramma accesi di serie: `Log/lio_timing.csv`, `Log/vio_timing.csv` nella cartella della run (`fast_livo2/`) |
| `common_lib.h` | file di lavoro in `$C2X_LIVO_DIR` (lo script lo mette nella cartella della run) invece che nei sorgenti |
| `vikit_common`, Sophus | opzioni solo x86 tolte su aarch64; configurazione CMake con percorsi relativi; niente registrazioni in `~/.cmake/packages` |
| launch, `run_multi_exploration.sh` | `ODOM_SOURCE=livo`: FAST-LIVO2 + `parameter_blackboard` (parametri della telecamera per vikit) al posto di FAST-LIO, stessi topic in uscita; telecamera a `LIVO_FOTO_HZ` (di serie 10) da una COPIA del modello nella cartella della run, il modello di PX4 resta a 5 Hz per le altre run |

**Come si e' arrivati alla configurazione attuale** (parco, `VISITA=0`, 1.0 m/s, Jetson;
`scripts/confronta_stima.py` per la stima, `scripts/analisi_campagna.py` per il volo):

1. Prima versione (18 ottobre, run `..._1924`): telecamera a 5 Hz, quindi stima verso PX4 a
   5 Hz con 0.16 s di ritardo; missione 310 s, margine 0.42 m. Una run con `VISITA=1`
   (`..._1909`) e' finita in un crash dopo un contatto a 0.40 m.
2. Telecamera a 10 Hz: due crash (`..._0000`, `..._0010`) e una divergenza al rientro
   (`..._0019`). I tempi per fotogramma (`lio_timing.csv`) hanno mostrato che il calcolo
   stava nei tempi (LIO 30-56 ms, VIO 14-30 ms) ma ogni fotogramma partiva fino a 0.9-1.4 s
   dopo l'arrivo dei dati: aspettava l'IMU, rimasta in coda per `spin_some`. Con `spin_all`
   nessun crash nelle 7 run successive.
3. Errore di quota di ~11 cm (4 run): non era la telecamera (stesso errore col solo lidar,
   `img_en: false`, run `..._0041`), ne' la mappa (celle a 0.25 m peggio: 13 cm, `..._0048`),
   ne' il rumore dichiarato dell'IMU (0.1 peggio: 10 cm, `..._0056`). Era l'allineamento alla
   gravita' (`uav.gravity_align_en`): la gravita' stimata all'avvio dalla media
   dell'accelerometro contiene l'errore costante dell'IMU simulata (0.05 m/s2), e il mondo
   veniva ruotato di 0.35-0.39 gradi in tutte le run; il pavimento della mappa 3D usciva
   inclinato di 0.3-0.45 gradi. Spento: inclinazione 0.16 gradi, quota 4-6.5 cm.

**Risultato con la configurazione attuale** (mediane; FAST-LIVO2: 3 run `2026-10-19_0149`,
`_0204`, `_0257`; FAST-LIO: 6 run, `2026-10-18_1933`, `2026-10-19_0114`, `_0130` e le
`2026-10-18_1644/1652/1701` di Drone_20261017_P, stesso codice):

| | FAST-LIVO2 | FAST-LIO |
|---|---|---|
| run finite senza crash | 3/3 | 6/6 |
| missione (decollo-appoggio) | **258 s** (216-267) | 273 s (249-322) |
| margine minimo dagli ostacoli | 0.36 m (0.20-0.46, **un contatto**) | 0.47 m (0.40-0.51) |
| quota minima vera in crociera | 0.54 m | 0.72 m |
| errore all'atterraggio | 0.04 m | 0.04 m |
| mappa 3D colorata | 90.7% | 90.6% |
| errore orizzontale proprio (tolto il ritardo) | 7.1 cm | 3.0 cm |
| errore di quota | 5.4 cm | 1.1 cm |
| pavimento della mappa 3D | +7 cm, inclinato 0.16 gradi | -2 cm, 0.09 gradi |
| stima verso PX4 | 10 Hz, 0.08-0.13 s | 10 Hz, 0.08-0.10 s |
| CPU dello stimatore | ~78% di un core | ~71% |

**Conclusione: FAST-LIO resta lo stimatore di serie.** FAST-LIVO2 ora vola stabile, finisce
le missioni piu' in fretta (-5%) con la stessa copertura delle foto e lo stesso errore
all'atterraggio, ma stima peggio la posizione (orizzontale 2 volte, quota 5 volte), vola piu'
basso del previsto e passa piu' vicino agli ostacoli: non soddisfa "le prestazioni non devono
peggiorare". Resta disponibile con `ODOM_SOURCE=livo`; tornare a FAST-LIO non richiede nulla
(`ODOM_SOURCE=lio`, di serie in tutti i comandi del LEGGIMI). Il lidar simulato e' perfetto e
il parco e' pieno di superfici: il vantaggio di FAST-LIVO2 andrebbe cercato con sensori veri
o in ambienti poveri per il lidar (corridoi, campi aperti).

**Se si riprende.** Errore orizzontale e di quota residui: l'errore costante
dell'accelerometro anche sull'asse z, il peso dell'aggiornamento visivo (`vio.img_point_cov`),
e piu' run per tipo (la variabilita' fra run e' grande: 4-13 cm di quota con la stessa
configurazione). Per confronti fini conviene registrare un volo (lidar, IMU, telecamera,
verita' di Gazebo) e rigiocarlo su entrambi gli stimatori: il registratore deve usare il
profilo DDS del progetto, altrimenti rallenta le nuvole lidar (cosi' e' fallita la prova
`..._2113`).

---

## Collaudo del drone vero (19 ottobre 2026)

Per le prime prove sul drone vero (al chiuso, senza GPS, senza radiocomando, Pixhawk
collegato al Jetson via USB, Mid-360 e telecamera) c'e' `ros2_ws/scripts/collaudo/`: un
programma a fasi (`collaudo.py`) che va dalle verifiche a terra a un quadrato automatico in
offboard, facendo da radiocomando (levette virtuali da tastiera o joystick su
`/fmu/in/manual_control_input`), con arresto d'emergenza e atterraggi automatici. Procedura,
preparazione e sicurezza in **`ros2_ws/scripts/collaudo/LEGGIMI_COLLAUDO.md`**; la lista di
controllo in ordine per la campagna di prove in `ros2_ws/scripts/collaudo/PROCEDURA_PROVE_REALI.md`.

```bash
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh sim          # prova a casa, in simulazione
cd ros2_ws/scripts/collaudo && ./avvia_collaudo.sh reale /dev/ttyACM0
```

Provato in simulazione (PX4 SITL + Gazebo + FAST-LIO, `SOLO_STIMA=1`, `cubicle_office`): fasi
0-7 superate, levette virtuali accettate da PX4 come comando "MAVLink", emergenza in 21 ms;
posizione stimata contro quella vera: hover 0.8 cm (max 2.2), volo da tastiera 1.7 cm (max
6.2), quadrato offboard 1.7 cm in mediana. Cose emerse provando, e gia' corrette:
- decollo in POSCTL con la levetta del gas: PX4 si dichiarava decollato ma il drone restava
  appoggiato 10-20 s (spinta che recupera piano dopo la rampa) e poi si staccava di colpo;
  un quadrato partito cosi' l'ha ribaltato. Ora il decollo e' in offboard con la quota che
  sale a 0.4 m/s; il quadrato parte solo alla quota giusta; motori spenti se dopo 12 s il
  drone e' ancora sotto 10 cm. Lo stesso rischio sul drone vero se `MPC_THR_HOVER` e'
  troppo basso (il modello simulato ha 0.60, la spinta vera di hover e' 0.75);
- in `empty_room` (stanza quasi vuota) FAST-LIO **non ha visto** uno spostamento vero di
  4.8 m con i numeri di salute buoni: il luogo di volo deve avere struttura vicina, e la
  fase 3 (drone spostato a mano) va fatta nel punto di decollo.
Driver del Mid-360 installato (19 ottobre): Livox-SDK2 dentro il progetto
(`terze_parti/livox_sdk2`) e `livox_ros_driver2` completo nello spazio di lavoro (vedi
LEGGIMI_COLLAUDO). Da fare prima del volo vero: rete del lidar (192.168.1.x come il Wi-Fi del
Jetson), DDS sull'USB del Pixhawk (`extras.txt`, da verificare).

**Correzioni allo script delle run fatte in questa occasione** (valgono per tutte le run):
`SOLO_STIMA=1` (solo Gazebo, PX4, sensori, stimatore e ponte: niente pianificatore, niente
archivio; l'airframe 22000 con `PX4_COLLAUDO=1` applica i limiti di `collaudo_px4.params` e la
spinta di hover vera 0.75, solo in questa modalita'); tempo
limite di 20 s su `ros2 daemon stop` (due volte e' rimasto fermo 6 e 46 minuti); tempo
limite di 60 s sul salvataggio delle mappe (dopo un crash aspettava per sempre); attesa del
registratore della mappa fino a 300 s e salvataggio atomico (`registra_mappa.py`): con 16
milioni di voxel la mappa 3D partiva su un file scritto a meta' e falliva in silenzio; gli
errori della ricostruzione ora si vedono (`ricostruisci_<N>.log`).

---

## Missione di esplorazione sul drone vero (19 ottobre 2026)

Lo stesso stack di `run_multi_exploration.sh` (stessi nodi, stessi parametri del
pianificatore) per il drone vero, senza simulazione:

```bash
cd ros2_ws
BOX="-1.5,-2,4,2" ./scripts/run_reale_exploration.sh /dev/ttyACM0      # drone vero
BOX="-1.5,-6,10,6" ./scripts/run_reale_exploration.sh sim               # prova generale a casa
```

**`BOX` e' obbligatorio**: il recinto della missione in metri, nelle coordinate di FAST-LIO,
cioe' del drone al momento dell'avvio della stima: origine nel punto di decollo, **x in avanti**
(dove punta il muso), **y a sinistra**. Il decollo deve stare ad almeno 1 m dai bordi. Valori di
serie per le prime prove: `VEL_MAX=0.3`, `TEMPO_MAX=240` (s di esplorazione, poi rientro),
`TEMPO_VOLO_MAX` = TEMPO_MAX + 180 (poi atterra dov'e'), `VISITA=0`, `QUOTA_ZONA=2.2`. Tutte le
variabili sono descritte in testa allo script. Registri in `/tmp/c2x_run_reale`, archivio in
`risultati_prova/<data>_reale_lio_v0.3/` (con `sim`: `_reale-sim_<mondo>_...`, piu' mondo
simulato e `.ulg`, leggibile da `analisi_campagna.py`). Sul drone vero il `.ulg` resta sulla
scheda SD del Pixhawk.

Sequenza: verifiche (porta, agente, driver, **nessun processo di sessioni precedenti**, che
non vengono uccisi: potrebbero tenere in aria il drone) -> agente DDS seriale -> driver Livox
-> **stima** (FAST-LIO + ponte verso PX4) -> **controlli a terra** -> **consenso** (si scrive
`VOLA`) -> **missione** (il ponte dei comandi arma e decolla subito a 1 m, giro di 360 gradi,
via all'esplorazione, rientro, atterraggio sul punto di decollo) -> mappe, spegnimento con la
stima per ultima e solo a drone disarmato, archivio.

I pezzi:
- `multi_drone_exploration.launch.py`: argomento **`reale:=true`** (un drone solo, solo
  `odom_source:=lio`): niente `/clock`, niente ponti Gazebo, niente `lidar_lio_adapter`, tutto a
  tempo reale; FAST-LIO con `scripts/collaudo/fast_lio_mid360.yaml` sul driver
  (`/livox/lidar` CustomMsg, `/livox/imu`); la nuvola per mappa e filtro di sicurezza e' la
  scansione di FAST-LIO in terna corpo (`/cloud_registered_body` -> `/drone_1/lidar/points`),
  gia' raddrizzata e con lo **stesso istante** della stima, la coppia che
  `cloud_transform_bridge` cerca; `lidar_offset` = posizione del Mid-360; modello di visibilita'
  del pianificatore col campo vero del Mid-360 (+52/-7 gradi). Argomento **`parte:=stima|missione`**
  (di serie `tutto`): due lanci separati, perche' il ponte dei comandi arma e decolla appena
  parte e fra i due vanno i controlli e il consenso. `C2X_SOLO_STIMA=1` equivale a
  `parte:=stima`. In simulazione i nodi generati sono **identici** a prima (confrontati per
  pacchetto, nome, parametri, rimappature su sei combinazioni: lio/px4/livo, 1-3 droni,
  `SOLO_STIMA`, variabili d'ambiente).
- `scripts/collaudo/prepara_reale.py`: configurazione dal recinto (stessa base delle
  simulazioni, `config_generica` di `prepara_mondo.py`, piu' il tetto `fsm.max_explore_time`).
- `scripts/collaudo/sorveglia_missione.py` (usa la classe `Drone` di `collaudo.py`):
  `controlli` (fasi 0-7 del collaudo **sul drone vero** superate - `SENZA_COLLAUDO=1` per
  saltarlo, resta scritto -, PX4 collegato e disarmato e a terra, FAST-LIO a >= 8 Hz, ferma
  entro 5 cm, >= 500 punti agganciati, nell'origine, PX4 d'accordo con FAST-LIO entro 20 cm,
  EKF2 che fonde la visione, batteria >= 50%); `volo`, in primo piano per tutta la missione:
  **SPAZIO = emergenza** (motori spenti, stack della missione spento), **`l` = atterra**, e
  atterraggio da solo se: stima ferma da 1 s, < 300 punti agganciati, posizione di PX4 non
  valida, fuori dal recinto di oltre 0.5 m o sopra 2.3 m, batteria < 15%, esplorazione non
  partita un minuto dopo il decollo o nessuna frontiera al via, tempo di volo scaduto, stack
  della stima o della missione terminati; SIGHUP (sessione SSH caduta), SIGTERM e Ctrl+C =
  atterraggio, e il programma resta attivo fino al disarmo. Se il decollo non e' concluso
  spegne anche lo stack della missione (il ponte riarmerebbe un drone disarmato).

**Difetti trovati con la prova generale e corretti** (valgono anche per la simulazione):
- **Rientro impossibile sotto ~0.55 m/s.** `KinodynamicAstar` espande con accelerazioni a passi
  di `max_acc/2` per `max_tau` = 0.8 s: salti di velocita' di 0.8 m/s, oltre `max_vel` +
  `vel_margin` (0.3 + 0.25). Da fermo ogni primitiva veniva scartata ("Kino replan: Can't find
  path", 3 nodi): 109 pianificazioni fallite e drone atterrato allo scadere del rientro a 2.9 m
  dalla partenza (run `2026-10-19_0451_reale-sim...`). Il launch ora abbassa
  `search.max_acc` della sola ricerca quanto basta (1.31 a 0.3 m/s); a 0.6 m/s e oltre non
  cambia nulla. Dopo: rientro pianificato e atterraggio a 3 cm dal decollo.
- **Atterraggio conteso.** Un NAV_LAND mandato da fuori con il ponte dei comandi attivo: il ponte
  continua a pubblicare `/fmu/in/trajectory_setpoint`, che in PX4 e' lo stesso topic in cui
  scrive il compito di atterraggio; nel `.ulg` i setpoint si alternano ("scendi" e "resta a
  0.77 m") e PX4 non ha riconosciuto l'atterraggio per **60 s con i motori accesi**. Il
  sorvegliante ora chiede l'atterraggio al ponte su `land_request` (come `lio_to_px4_bridge`:
  il ponte smette di pubblicare e manda lui NAV_LAND), con NAV_LAND diretto di riserva, e
  spegne la missione se dopo 1 s il ponte manda ancora setpoint. Dopo: da 1.07 m a terra in
  4 s, disarmo in 11 s. Lo stesso se e' **PX4 a passare da solo** in LAND/DESCEND/RTL
  (failsafe: batteria critica, stima persa): da 0.5 s in quella modalita' con il ponte che
  pubblica ancora, il sorvegliante gli manda `land_request` (atterraggio sul posto, anche al
  posto di un RTL che al chiuso non evita gli ostacoli). Provato con un NAV_LAND mandato a PX4
  da fuori in esplorazione (`..._1044_...`): ponte fermato dopo 0.5 s, a terra in 4 s,
  disarmo in 11 s.
- **Atterraggio conteso, corretto anche nel ponte dei comandi** (19 ottobre 2026, vale anche
  per le run simulate, che non hanno sorvegliante). Prima il ponte smetteva solo per gli
  atterraggi chiesti a lui, e quando PX4 usciva da solo dall'offboard continuava a pubblicare.
  Ora `position_command_bridge` legge `nav_state` da `vehicle_status_v1` (vedi
  `controllaModalitaPx4()`): con il drone armato, dopo averlo visto armato in OFFBOARD (nella
  prima versione solo a decollo concluso, vedi il punto seguente), se PX4 e' in una
  modalita' qualsiasi diversa da OFFBOARD (LAND, DESCEND, RTL, HOLD...: failsafe, NAV_LAND di
  terzi, cambio di modalita' da QGroundControl), il ponte **smette per sempre** di pubblicare
  setpoint e battito, chiude l'esplorazione (`flight_terminated` all'FSM: "VOLO INTERROTTO") e
  non riarma. `land_request` vale ancora (manda NAV_LAND: da un RTL o da un HOLD si atterra sul
  posto), `land_at_home` no. Prima dell'armamento in offboard non cambia nulla: li' il battito
  serve a PX4 per entrare in offboard. Nei 116 `.ulg` archiviati (Drone_20261017_P e _18_P) PX4
  non e' mai uscito da solo dall'offboard in volo, quindi il volo normale non cambia. L'unico
  comportamento nuovo e' con un buco del battito oltre `COM_OF_LOSS_T`: PX4 passa in LAND e
  prima, alla ripresa del battito, sarebbe tornato in offboard; ora atterra. Il controllo del
  sorvegliante resta come riserva, ma interviene solo se il ponte pubblica ancora, quindi un RTL
  scelto da PX4 non viene piu' trasformato in atterraggio sul posto. Con `collaudo_px4.params`
  pero' tutti i failsafe sono gia' atterraggi (`NAV_RCL_ACT` 3, `COM_OBL_RC_ACT` 4,
  `COM_LOW_BAT_ACT` 2): un RTL arriva solo se lo chiede qualcuno.
  Verifiche (Jetson): run di non regressione `ODOM_SOURCE=lio VISITA=0 PF_MIN_RAM_GB=3
  ./scripts/run_multi_exploration.sh parco 1` pulita (`2026-10-19_1119`: esplorazione conclusa,
  rientro e atterraggio a 2 cm, missione 210 s, margine vero 0.50 m, quota minima 0.75 m, errore
  di stima 6 cm; le run 0435 e 0400: 276/279 s, 0.54/0.36 m, 0.63/0.72 m). Un NAV_LAND mandato
  a PX4 da fuori in esplorazione, **senza sorvegliante**, a 1.3 m di quota e 1 m/s
  (`2026-10-19_1132`): il ponte si ferma appena PX4 passa in LAND (ultimo setpoint 0.03 s dopo
  il comando), a terra in 2.2 s (inclinazione massima 11 gradi, 0.5 m di deriva nella
  direzione di marcia), disarmo in 5.2 s, la run si chiude da sola.
  **Attenzione: l'atterraggio sul posto di PX4 non vede gli ostacoli.** Nel primo tentativo
  (`2026-10-19_1125`) il drone stava ispezionando l'utilitaria rossa da 0.75 m: e' sceso sulla
  carrozzeria, e' scivolato a terra e si e' ribaltato (poi stima di FAST-LIO divergente e
  disarmo dopo 39 s). Il ponte non c'entra (fermo in 0.2 s, nessun setpoint dopo il LAND): vale
  per ogni failsafe di atterraggio, per un NAV_LAND da fuori e per il tasto `l` del sorvegliante
  (`land_request` = atterraggio sul posto). La seconda prova e' stata fatta mandando il comando
  solo con il terreno libero entro 1.5 m sotto il drone.
- **Cedimento esteso a salita e giro iniziale** (19 ottobre 2026, pomeriggio). Con la regola
  "a decollo concluso" salita e giro iniziale restavano scoperti: un failsafe LAND/DESCEND o un
  NAV_LAND di terzi in quella fase era ancora conteso, e al disarmo il ramo di riarmo del ponte
  (`(have_cmd_ || takingOff()) && !armed_`, con `takingOff()` ancora vero) mandava DO_SET_MODE
  offboard + ARM ogni 0.2 s e rimetteva in volo il drone dal suolo. Lo copriva solo il
  sorvegliante sul drone vero (stack spento se il decollo non si conclude). Ora la condizione e'
  "PX4 visto almeno una volta armato in OFFBOARD (`visto_offboard_armato_`, annotato a ogni
  messaggio di `vehicle_status_v1`), ora armato in un'altra modalita'"; dopo il cedimento
  `takingOff()` e' falso e il disarmo chiude la missione senza riarmo in qualunque fase ("[PX4]
  disarmato ... non lo riarmo"). Il messaggio di cedimento dice la fase: "in decollo", "nel giro
  iniziale", "in volo". Prima dell'armamento non cambia nulla: battito e setpoint di decollo
  come prima, stessa sequenza DO_SET_MODE + ARM con i suoi tentativi. Il primo stato armato e'
  sempre OFFBOARD (118 run armate su 118 nei `.ulg` di 17_P e 18_P), quindi la condizione si
  accende all'armamento; un armamento accettato in un'altra modalita' (offboard rifiutato) non la
  accende. Il ricordo serve perche' dopo il disarmo PX4 torna da solo in OFFBOARD (la modalita'
  di prima del LAND): a disarmo avvenuto lo stato non dice piu' che c'e' stato un LAND.
  Verifiche (Jetson, senza sorvegliante): NAV_LAND da fuori **nel giro iniziale**, 3 s dopo
  l'inizio del giro a 0.95 m (`2026-10-19_1320_..._interrotta`): ponte ceduto subito ("uscito da
  solo dall'offboard nel giro iniziale"), nel `.ulg` battito fermo all'istante del LAND e dopo
  il LAND solo i setpoint del compito di atterraggio di PX4, a terra entro 1.6 s, disarmo 7.0 s
  dopo il LAND, nessun DO_SET_MODE ne' ARM dopo il NAV_LAND, disarmato per tutti i 60 s
  osservati. NAV_LAND **in salita** a 0.57 m (`2026-10-19_1325_..._interrotta`): ultimo setpoint
  e ultimo battito del ponte 0.03 s dopo il comando, nessuno dopo il LAND, a terra in ~1.4 s,
  disarmo dopo 6.8 s, nessun riarmo ne' comando del ponte in 60 s. In entrambe lo script chiude
  per handshake di prontezza scaduto (120 s, "decollo / giro iniziale in corso": l'FSM in FINISH
  non aggiorna piu' la prontezza) e archivia la run come `_interrotta`.
  Non regressione `ODOM_SOURCE=lio VISITA=0 PF_MIN_RAM_GB=3 MOSTRA_MAPPA=0
  ./scripts/run_multi_exploration.sh parco 1`, confrontata con `analisi_campagna.py` alla 1119:
  `2026-10-19_1328` pulita (missione 238 s contro 210, margine vero 0.47 contro 0.50 m, quota
  minima 0.72 contro 0.75 m, errore di stima 9 contro 6 cm, atterraggio a 3 contro 2 cm da casa,
  foto 91.0 contro 90.8%). Prima due run non pulite, con PX4 in OFFBOARD fino al LAND chiesto al
  ponte (il cedimento non e' mai scattato: il codice nuovo non e' entrato in gioco):
  `2026-10-19_1300` urto e ribaltamento contro l'area giochi dopo 140 s di esplorazione (drone
  a 0.26 m dalla geometria vera in (-12.5, 9.7, 1.5), poi stima di FAST-LIO divergente e LAND),
  `2026-10-19_1310` conclusa e atterrata (231 s, 6 cm da casa) ma con margine 0.30 m, sempre
  all'angolo sud-ovest dell'area giochi (-14.6, 9.7, 1.4). Anche il punto piu' vicino della
  1328 e' li' (0.47 m): un passaggio stretto da rivedere a parte.
  Resta scoperto: un disarmo **senza** cambio di modalita' prima della fine del decollo
  (emergenza del sorvegliante con PX4 in OFFBOARD, disarmo automatico di PX4 per decollo
  mancato) fa ancora riarmare il ponte. Sul drone vero il sorvegliante in quei casi spegne lo
  stack della missione: quel controllo resta necessario.
- Falsi allarmi del sorvegliante: sicurezze attive solo dalla prima stima ricevuta (il ponte
  armava prima che il sorvegliante ricevesse qualcosa) e stima ferma da **1 s**, non 0.5 (buco
  vero di 0.62 s all'avvio della missione, come gia' visto per `odom_timeout` del ponte).

Verifiche (19 ottobre 2026, Jetson): run di non regressione `ODOM_SOURCE=lio VISITA=0
PF_MIN_RAM_GB=3 ./scripts/run_multi_exploration.sh parco 1` pulita (`2026-10-19_0435`:
esplorazione conclusa, rientro e atterraggio, missione 276 s, margine vero 0.54 m, quota minima
0.63 m, errore di stima 8 cm; la run delle 04:00 prima delle modifiche: 279 s, 0.36 m, 0.72 m).
Lanci reali a vuoto (senza hardware): nodi, parametri (`use_sim_time` false, CustomMsg su
`/livox/lidar`, campo del Mid-360, `max_vel` 0.3) e topic come previsto. Prova generale
`cubicle_office`, recinto 11.5 x 12 m (`2026-10-19_1030_reale-sim...`): esplorazione conclusa,
rientro e atterraggio a 3 cm dal decollo, margine vero 0.80 m, quota minima 0.90 m, inclinazione
massima 8 gradi, errore di stima 5 cm, nessuna pianificazione fallita; interruzione in volo con
SIGTERM al sorvegliante (`..._1038_...`): atterraggio pulito, nessun riarmo.

**Cosa resta da provare sul drone vero** (in quest'ordine):
1. Collaudo, fasi 0-7 (`LEGGIMI_COLLAUDO.md`): i controlli a terra della missione lo pretendono.
   Comprende DDS sull'USB (`extras.txt`, mai provato su hardware), rete del lidar
   (192.168.1.x come il Wi-Fi del Jetson, IP in `MID360_config.json`), assi del lidar, motori.
2. **Misure del montaggio**: `PARTENZA_Z` (quota da terra del punto di riferimento del drone
   appoggiato) e `LIDAR_POS` (Mid-360 rispetto a quel punto, x avanti, y sinistra, z su).
   Sbagliate spostano la mappa di qualche cm. `EKF2_EV_POS_*` restano a zero (cosi' in
   `collaudo_px4.params`, come in simulazione): diversi da zero, PX4 inseguirebbe il baricentro
   mentre il pianificatore misura il lidar, e il drone volerebbe spostato di quella distanza.
3. **La nuvola di FAST-LIO come mappa**: in simulazione la mappa viene sempre dalla nuvola di
   Gazebo, questa sorgente non ha mai volato. Al primo avvio della stima guardarla in RViz
   (`/drone_1/lidar/points` e `/drone_1/map_ros/cloud`): punti sul telaio o sulle eliche
   (blind e `min_range` a 0.5 m), densita' (un punto su tre, `point_filter_num` 3).
4. **Campo del Mid-360 (+52/-7 gradi)**: il pavimento vicino non si vede mai e gli ostacoli
   bassi vicini neppure (sotto ~0.9 m entro ~1.5 m volando a 1 m). Il pianificatore ne tiene
   conto nel modello di visibilita', ma non e' mai stato provato: primo volo in uno spazio
   sgombro sotto 1 m, controllando che l'esplorazione si chiuda. Il riconoscimento
   chiuso/aperto (`[EnvSense]`) non funziona con questa nuvola: con un drone solo non conta.
5. **Velocita' vera**: con `VEL_MAX=0.3` il drone simulato ha toccato 0.59 m/s (media 0.29):
   per i primi voli `MPC_XY_VEL_MAX` 0.6 (gia' cosi' in `collaudo_px4.params`).
6. **Soffitto**: il rientro vola a 1.8 m (fisso nell'FSM) e il riferimento non supera 2.0 m:
   servono almeno 2.8 m di altezza libera.
7. Prima missione: recinto piccolo (4-5 m per lato) in un locale con struttura vicina,
   operatore a portata di tastiera (collegamento SSH corto: se cade, il sorvegliante fa
   atterrare). Poi `EKF2_EV_DELAY` misurato sul `.ulg` e, se tutto torna, recinti piu' grandi.
8. Telecamera: non collegata alla missione (mappa 3D grigia, dal solo lidar).

---

## Stato del lavoro

**Video dimostrativi (6 ottobre 2026).** In `risultati_prova/DEMO_investitore/`
(vedi `GUIDA_DEMO.md`): `demo_cubicle_office.mp4` (montato da `scripts/video_demo.py`
a partire da una run archiviata: pianta che si costruisce, spazio visitato,
telecamera di bordo, numeri dal vivo, mappa 3D in orbita, risultati) e
`demo_gazebo.mp4` (finestra di Gazebo registrata con `scripts/registra_demo_gazebo.sh`
durante un volo con la GUI, camera che segue il drone da ~14 m con un filtro di ~3 s,
fondo nero e traiettoria disegnati con i marker della GUI da
`scripts/camera_inseguimento.py` - il servizio /marker risponde gz.msgs.Empty -,
montaggio con `scripts/video_gazebo.py`). Versione attuale: solo esplorazione, 60 fps,
5x: `demo_esplorazione_5min.mp4` (mappa del LiDAR a sinistra) e `demo_volo_3d.mp4`,
ridisegnato con `scripts/video_replay3d.py` dai dati del volo (modello X500, posa
vera dal registro PX4, camera levigata): la camera della GUI di Gazebo guidata con
/gui/move_to/pose restava indietro di 6-9 m e andava a scatti, e con una correzione
in retroazione diventava instabile. Volo archiviato in `risultati_prova/DEMO_volo_6min`. Codifica
con GStreamer (x264enc + mp4mux): ffmpeg non e' installato. Nota: caricare l'ambiente
ROS prima di avviare la simulazione mette davanti nel PATH il `gz` ridotto di ROS
(senza `gz sim`) e Gazebo non parte; la GUI dedicata (`scripts/gui_demo.config`) deve
stare tutta nello schermo, altrimenti la cattura X fallisce.

**Visita completa dello spazio (campagne V31-V35, 6 ottobre 2026).**

L'esplorazione a frontiere finisce quando il LIDAR ha visto tutto, e il lidar vede
a 360 gradi e lontano: corridoi ciechi e stanze viste dalla porta risultano
esplorati senza che il drone ci entri. Misurato con la posizione VERA del
simulatore e la pianta vera del mondo (`scripts/copertura_visita.py`: spazio
percorribile alla quota di volo, a >= 0.5 m dagli ostacoli, visitato se il drone
ci e' passato entro 2 m con la vista libera), su cubicle_office la sola
esplorazione visita il 79-86% dello spazio (V31).

| dove | cosa |
|---|---|
| `copertura_visita.{h,cpp}` + FSM | **fase di visita** (`fsm.visit_completion`, di serie accesa): finite le frontiere, lo spazio percorribile mai visitato (pianta del drone alla quota di volo, celle entro `fsm.visit_radius` = 2 m dal volo con vista libera) viene raggruppato in zone da almeno 1 m2; il drone le percorre in ordine di giro (vicino piu' vicino + 2-opt) e passa alla successiva appena la meta risulta visitata, senza fermarsi. Tetto di tempo in comune con la fase fotografica (`fsm.photo_completion_time`, 300 s) |
| `run_multi_exploration.sh`, launch | `VISITA=0` toglie la fase di visita (solo esplorazione); `FASE_FOTO=1` riaccende la fase fotografica (di serie spenta); `IMBARDATA_FOTO=1` riaccende l'imbardata da foto (di serie spenta: con la visita la telecamera in avanti fotografa gia' la mappa 3D, e la rotazione - ~2400 gradi/min contro ~900 - rallentava la visita) |
| `planner_manager.cpp` (`planPolilinea`) + FSM (V34) | **piano di riserva**: se il drone e' fermo e il piano ottimizzato taglia un angolo entrando nella fascia di gonfiaggio, il drone percorre la spezzata del percorso (tratti gia' verificati liberi) fermandosi ai vertici, a 0.5 m/s. Senza, un drone fermo nella stanza a "П" ha fallito 30 piani di fila (V33) |

| cubicle_office, 1.0 m/s, senza GPS | spazio visitato (posizione vera) | mappa 3D mai fotografata | missione | esito |
|---|---|---|---|---|
| solo esplorazione (V31, 3 run) | 78.9-86.2% | 2.0-4.5% | 313-342 s | 3/3 pulite |
| esplorazione + fase fotografica (V30, 2 run) | 88.7-96.2% | 1.0-2.4% | 567-648 s | 2/2 pulite |
| esplorazione + visita, imbardata da foto (V32, 3 run) | 94.4-99.5% | 0.6% (run 1) | 578-619 s | 3/3 pulite |
| esplorazione + visita, imbardata da foto (V33, 2 run) | 89.9% | 5.6% | 598 s | 1 ribaltamento in esplorazione, 1 pulita |
| esplorazione + visita, telecamera in avanti (V33, 2 run) | 97.7-99.4% | 1.6-2.7% | 509-758 s | 2/2 pulite |
| **di serie (V34, 3 run)** | **96.3-99.3%** | 3.2-4.7% | **526-549 s** | **3/3 pulite**, margine 0.54-0.67 m |
| di serie, office_small (V34, 1 run) | 94.8% | foto% 95.4 | 118 s | pulita |
| **di serie con V35, cubicle_office (2 run)** | **99.5-99.9%** | foto% 97.4-97.6 | **509-577 s** | **2/2 pulite**, margine 0.42-0.60 m |
| di serie con V35, office_small (1 run) | 99.3% | foto% 92.9 | 111 s | pulita |

Senza fase fotografica la mappa 3D di office_small resta fotografata al 93-95%
(99.6% con `FASE_FOTO=1`, missione ~170 s invece di ~115). Immagini della visita:
`risultati_prova/V35_*/visita_run*.png` (rosso = spazio mai visitato).

Le zone visitate non si scartano (V35): in una zona lunga, come una striscia
lungo una parete, la meta copre solo i 2 m attorno, e scartandone il centro i
pezzi rimasti venivano ignorati (20 m2 lungo la parete est di cubicle_office in
una run V34).

**Dondolio, velocita' e mappa 3D completa (campagne V20-V30, 5-6 ottobre 2026).**

| dove | cosa |
|---|---|
| `active_perception/cache_percorsi.h`, `uniform_grid.cpp`, `hgrid.cpp` | **memoria dei percorsi** per il giro delle celle: l'esito di ogni ricerca A* fra due centri si riusa nelle pianificazioni successive (percorso ricontrollato cella per cella sulla mappa attuale, al massimo 8 s se trovato e 4 s se fallito, estremi quantizzati a 25 cm). Pianificazione su cubicle_office da ~1.2 s a 0.3-0.45 s |
| `c2_exploration_fsm.cpp` (troncamento) | il segnale `planning/replan`, che fa finire la traiettoria in corso dopo ~0.3 s, si manda solo per scansare un ostacolo. A ogni ripianificazione fermava il drone ad aspettare il piano nuovo: era la causa del dondolio (riferimento fermo nel 23-47% dei comandi). Tolto anche a tempo massimo, a fallimenti di fila, a "nessuna frontiera" e nel controllo di avanzamento: a fine esplorazione il drone frenava di colpo e oscillava |
| `c2_exploration_fsm.cpp` (piano in ritardo) | un piano che parte dalla traiettoria in corso e arriva in ritardo viene preso gia' avviato, nel punto in cui il drone e' arrivato; se li' dista piu' di 25 cm dalla traiettoria in corso si scarta e si ripianifica (V22: un salto di 0.9 m del riferimento ha mandato il drone contro un ostacolo). Un piano dalla fine della traiettoria (drone fermo sul punto di vista) parte subito |
| `c2_exploration_fsm.cpp` (velocita' di partenza) | lo stato iniziale di un piano si limita a `max_vel`/`max_acc`: un drone in sovravelocita' ripianificato dalla velocita' misurata la conservava piano dopo piano (V24: 1.9-3 m/s con tetto 1.0, poi contro una parete) |
| `c2_exploration_fsm.cpp` + `traj_server.cpp` (V27) | **piano dallo stato misurato preso all'istante giusto**: partiva "adesso" dalla posizione di 0.6-1.1 s prima, quindi con il riferimento alle spalle del drone; il server di traiettoria leggeva la distanza come un ritardo e lo rallentava ancora, e il piano seguente ripartiva di nuovo dallo stato misurato. Ora l'origine dei tempi e' l'istante della misura (presa nel punto del piano piu' vicino al drone), la soglia per ripartire dallo stato misurato confronta il drone con il riferimento di adesso (non con quello 0.2 s avanti) e il server rallenta solo se il drone e' indietro |
| `planner_manager.cpp` + FSM (V28) | **nessuna traiettoria in collisione eseguita**: ogni piano nuovo si controlla contro la fascia di gonfiaggio (stessa regola del controllo periodico) prima di pubblicarlo; se e' in collisione si tiene la traiettoria in corso, oppure, se anche quella non e' sicura, si **frena in linea retta** dallo stato misurato (2 m/s2). Le ripianificazioni anti-collisione partono dallo stato misurato, non dalla traiettoria in collisione. Prima un piano in collisione veniva eseguito e ripianificato da se' stesso fino a 7 volte in 1.5 s verso la parete: 2 ribaltamenti su 2 run in V27 a 1.0 m/s |
| `traj_server.cpp` (V29) | oltre la fine di una traiettoria (o dopo un troncamento) che ha ancora velocita', il riferimento frena a `traj_server.brake_acc` invece di fermarsi di colpo: con il gradino il drone superava il riferimento di 0.7 m (0.32 m da una parete in V28) |
| `planner_manager.cpp` + `copertura_camera.cpp` (V29) | **imbardata da foto** (`fsm.photo_yaw`): durante l'esplorazione, nei punti di passaggio dell'imbardata si sceglie fra 12 direzioni quella da cui la telecamera inquadrerebbe piu' superfici non ancora fotografate (raggi radi nel campo visivo), con un piccolo vantaggio per la direzione di marcia. Il lidar vede a 360 gradi, quindi la sicurezza non cambia. Superficie inquadrata a fine esplorazione su cubicle_office: 76% -> 85-86% |
| FSM, fase fotografica (V29-V30) | controllo d'arrivo che aspetta il drone (il tempo della traiettoria scadeva con il drone ancora a 0.5-1 m dalla meta: 19 zone su 31 perse); zone scelte per resa (celle / (distanza + 3 m)) invece che per vicinanza |
| `exploration_manager/copertura_camera.{h,cpp}` + FSM | **fase fotografica**: il lidar vede a 360 gradi e l'esplorazione finisce quando ha visto tutto, ma la telecamera ne vede 90 e il 10-15% della mappa 3D restava grigio. Si tiene il conto delle superfici davvero inquadrate (raggi dal centro ottico nel campo visivo); finite le frontiere, le zone mancanti (fra -0.15 e 2.0 m di quota) vengono raggruppate e il drone va a guardarle una per una prima di rientrare (`fsm.photo_completion`, al massimo `fsm.photo_completion_time` = 300 s) |
| `ricostruisci_mappa.py` | fino a 1000 fotogrammi (prima 250, scelti a intervalli regolari: si perdevano proprio quelli delle ultime zone); tolleranza del controllo di occlusione che cresce con la distanza |
| `run_multi_exploration.sh` | attesa del rientro 450 s (era 120): con la fase fotografica in mezzo le run venivano chiuse a meta' rientro |
| `analisi_campagna.py` | nuove colonne: `foto%` (superficie di pavimento e pareti sotto 2 m colorata dalle foto nella mappa 3D), `rif0%` (comandi a velocita' zero), `dv` (errore di velocita'), `miss.s`, `casa m` |

Misura del dondolio (`dondolio.py`, dal log PX4): inversioni del verso del moto al
minuto, episodi di andata e ritorno (8 s con >= 2 m percorsi e spostamento netto
< 25%), quota del tempo in moto con il riferimento piu' di 0.3 m alle spalle del drone.

| | inv/min | episodi | rif. alle spalle | esito |
|---|---|---|---|---|
| cubicle_office 1.0 m/s, run dell'utente (codice V25) | 10.7 | 7 | 20.6% | conclusa |
| cubicle_office 0.7 m/s (V25) | 3.6 | 5 | 3.7% | pulita |
| office_small 1.0 m/s (V26, 3 run) | 5.4-7.6 | 2-5 | 7.7-12% | 3/3 concluse senza contatti, mappa 3D fotografata al 99.3-99.6% |
| cubicle_office 1.0 m/s (V27, prima dei ribaltamenti) | 3.7-5.1 | | 0-4.4% | 2 ribaltamenti su 2: piani in collisione eseguiti (corretto in V28) |
| cubicle_office 1.0 m/s (V28, 1 run) | 7.5 | 19 | 7.8% | pulita, margine minimo 0.32 m (frenata a gradino a fine traiettoria, corretta in V29), mappa 3D fotografata al 94.4% |
| cubicle_office 1.0 m/s (V29, 3 run) | 4.0-4.7 | 11-17 | 3.4-3.9% | 3/3 pulite, margine 0.45-0.60 m, mappa 3D fotografata al 94.5-96.8%, missione 596-642 s |
| office_small 1.0 m/s (V29, 2 run) | 3.0 | 4-7 | 0.4-2.0% | 2/2 pulite, margine 0.50-0.75 m, fotografata al 99.7%, missione 146-167 s |
| **cubicle_office 1.0 m/s (V30, 2 run)** | **2.4-3.0** | 11-18 | **0.7-1.7%** | **2/2 pulite, margine 0.51-0.60 m, inclinazione max 15-17 gradi, copertura pareti e precisione 100%, mappa 3D fotografata al 97.6-99.0%, missione 567-648 s, atterraggio a 5-6 cm** |
| **office_small 1.0 m/s (V30, 1 run)** | 4.1 | 4 | 1.0% | pulita, margine 0.50 m, fotografata al 99.6%, missione 159 s |

Gli "episodi" su cubicle_office sono soprattutto i cambi di meta della fase
fotografica (zona dopo zona) e i vicoli ciechi, non piu' il dondolio sul posto.

Restano: picchi di velocita' vera fino a 1.7-1.9 m/s quando il pianificatore
inverte la meta mentre il drone va a 1 m/s (il drone supera il riferimento e poi
lo rincorre); la fase fotografica su cubicle_office arriva al tetto dei 300 s
(la missione dura ~10 minuti, di cui ~5 di esplorazione). Strumenti:
`scripts/dondolio.py` (misure qui sopra), `analisi_campagna.py --mondo
cubicle_office --spawn=-13.5,-4`.

**cubicle_office e dondolio (campagne V17-V19, 5 ottobre 2026).** Il drone oscilla
avanti e indietro perche' a ogni ripianificazione il server di traiettoria tronca
quella in corso dopo 0.4 s, e su cubicle_office il piano nuovo arriva dopo 1-3 s
(giro delle celle: grafo di connettivita' + matrice dei costi, crescono con la mappa;
su office_small 0.3 s). Il drone frena, aspetta e rincorre il riferimento che riparte
da dietro. Misure:

| | inversioni accelera/frena al minuto | beccheggio | esito |
|---|---|---|---|
| office_small 0.9 m/s (V16) | 7 | 2.8 gradi | 4/4 pulite |
| office_small 1.0 m/s (V19) | 10-18 | 2.8-3.1 gradi | 2/2 pulite, missione 78-86 s, copertura 99.8-100% |
| cubicle_office 1.1 m/s (run manuale) | 54 | 8.7 gradi | conclusa |
| cubicle_office 0.7 m/s (V19) | 56 | 6.1-6.2 gradi | 2/2 pulite, missione 12-13 min, copertura 98-99%, precisione 100% |

Provati e SCARTATI (tutti peggiorano o causano crash su cubicle): anticipo adattivo
senza troncamento; anticipo fisso piu' lungo (0.45/2.0 s); celle da 10 m; limite A*
0.01 s e passo di partizione 2 su cubicle. Nelle prove a 1.0 m/s su cubicle il drone,
in ritardo di ~1 m sul riferimento nei corridoi stretti, ha toccato ostacoli, e
FAST-LIO ha avuto salti di 3-6 m (pianta piena di box identici). Per togliere il
dondolio su cubicle va reso piu' veloce il giro delle celle (es. riusare il grafo di
connettivita' fra una pianificazione e l'altra): fatto in V20-V30, vedi sopra.

Campagna V16 (5 ottobre 2026, `risultati_prova/V16_*`): planner corretto, 0.9 m/s,
`office_small`, un drone.

| | senza GPS (4 run) | con GPS (3 run) |
|---|---|---|
| concluse e atterrate, nessun contatto | 4/4 | 3/3 |
| durata della missione (decollo-appoggio) | 72-85 s (mediana 79) | 87-114 s (mediana 96) |
| copertura pareti visibili, mappa 3D | **99.9-100%** | 94-96% (filtro "visti 2 volte") |
| precisione pareti, mappa 3D | 100% | 93.5-96.9% |
| margine minimo dalle pareti | 0.55-0.72 m | 0.65-0.76 m |
| atterraggio dal punto di decollo | 2-8 cm | 3-10 cm |

Velocita' col planner corretto (V15, senza GPS): 0.9 m/s -> missione 88-91 s,
margine 0.60-0.81 m; 1.2 m/s -> 74-82 s ma margine 0.40-0.70 m: a 1.2 m/s, nelle
curve, il controllo di posizione di PX4 lascia il drone fino a 1 m fuori dal
riferimento. Per andare oltre 0.9 va ritarato il controllo di PX4, non il planner.

Giro iniziale a 0.8 rad/s invece di 0.5 (V16): ~5 s in meno ma superficie colorata
dalle foto 66-91% invece di 91-95%. Resta a 0.5.

Campagna V14 (5 ottobre 2026, `risultati_prova/V14_*`), `office_small`, un drone,
senza GPS, 3 run per configurazione, tutte pulite (12/12):

| configurazione | durata del volo | margine minimo dalle pareti | copertura 3D | precisione 3D | pendenza mappa | atterraggio dal decollo |
|---|---|---|---|---|---|---|
| 0.6 m/s, con giro | 136-148 s | 0.82-0.85 m | 98.7-99.0% | 100% | 0.03-0.06 gradi | 2-4 cm |
| 0.6 m/s, senza giro | 139-148 s | 0.76-0.85 m | 98.4-98.6% | 100% | 0.06-0.09 gradi | 3-11 cm |
| **0.9 m/s, con giro (default)** | **74-90 s** | **0.80-0.85 m** | **99.7%** | **100%** | 0.03-0.06 gradi | 5-6 cm |
| 1.2 m/s, con giro | 66-74 s | 0.65-0.70 m | 96.7-99.9% | 100% | 0.02-0.05 gradi | 9-12 cm |

Con GPS a 0.9 m/s (`V14_GPS_v09`): 3/3 concluse e atterrate, nessun contatto,
margine 0.80 m, volo 88-142 s, copertura 3D 93.9-98.6%, precisione 3D 95-97%,
atterraggio a 4-9 cm dal decollo: stessa qualita' di mappa che a 0.6 m/s (il limite
col GPS e' la bussola, non la velocita').

- **Velocita'**: 0.9 m/s dimezza il volo senza perdere nulla; 1.2 m/s toglie solo
  altri 8 s, avvicina il drone alle pareti di 15-20 cm e la velocita' vera arriva a
  1.7 m/s (sopra il tetto di PX4).
- **Giro iniziale**: non allunga il volo in modo misurabile e porta la superficie
  colorata dalle foto al 94-95% (senza: 58-89%, molto variabile). Il lidar vede gia'
  a 360 gradi, quindi la geometria cambia poco; e' la telecamera (90 gradi) a
  guadagnarci.
- **Perche' senza GPS la mappa e' migliore**: la posizione stimata e' precisa (~5 cm)
  in entrambi i modi; la differenza e' la DIREZIONE. Col GPS l'imbardata viene dalla
  bussola e sbaglia in media di ~2 gradi (fino a 3), che su una parete a 6 m sono
  20 cm: viste da punti diversi le pareti escono doppie. FAST-LIO ricava la
  direzione allineando ogni scansione alla mappa gia' costruita: 0.2-0.3 gradi. Nella
  realta' il divario sarebbe ancora piu' grande: il GPS civile sbaglia di 1-3 m e al
  chiuso non funziona proprio. Il GPS serve all'aperto e su lunghe distanze, dove la
  stima lidar accumula deriva.

Precisione della mappa nelle versioni precedenti (punti di parete entro 15 cm
da una parete vera):

| versione | con GPS | senza GPS |
|---|---|---|
| fino a V8 (posa piu' recente, bussola sfalsata) | 74-92% | 70-85% (strisciate) |
| V11 (posa dell'istante, bussola corretta) | 94% | 100% |

**Funziona:** decollo, esplorazione, rientro e atterraggio senza urti in entrambe
le modalita'; nessun crash del programma; i blocchi lunghi (fino a 616 s) sono
spariti - restano soste di 15-25 s che si sbloccano da sole.

**Limiti noti:**
- col GPS l'imbardata dalla bussola sbaglia di ~2 gradi e alcune pareti escono
  doppie (precisione 87-94% sulla mappa del planner, 96-97% su quella 3D);
- oltre 0.9 m/s il drone, nelle curve strette, si allontana fino a 1 m dal
  riferimento (controllo di posizione di PX4): i margini dalle pareti scendono a
  0.40 m a 1.2 m/s;
- brevi cicli di "ripianificazione per collisione" (20-25 s) quando la traiettoria
  ottimizzata taglia la fascia di gonfiaggio;
- con piu' droni la simulazione rallenta (0.35x con 3 droni) e lo stack ragiona
  sull'orologio di parete: da sistemare prima di riprendere il multi-drone.

**Da sapere prima di fidarsi della simulazione per l'hardware reale:**
il lidar simulato ha campo verticale 360°×100° simmetrico, mentre il Livox Mid-360
reale è 360°×59° **asimmetrico** (−7°/+52°). Il sensore vero vede molto meno verso
il basso: a 1 m di quota il pavimento gli compare solo da ~10 m, e ostacoli bassi a
3 m restano invisibili. Il modello va allineato prima di usare questi risultati per
decidere sull'hardware.
