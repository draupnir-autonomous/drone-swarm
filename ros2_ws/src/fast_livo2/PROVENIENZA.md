# FAST-LIVO2 in C2-Explorer: da dove viene e cosa e' cambiato

- **Originale**: https://github.com/hku-mars/FAST-LIVO2, commit `0d2c034` (8 marzo 2026),
  ROS 1. **Licenza GPLv2** (file LICENSE; il `package.xml` dichiara BSD, ma vale il file
  LICENSE). Per uso commerciale gli autori offrono una licenza alternativa (README).
- **Porting su ROS 2 (Humble/Jazzy) e correzioni**: le 5 patch di Intel, Robotics AI Suite
  (https://github.com/open-edge-platform/edge-ai-suites, ramo main,
  `robotics-ai-suite/pipelines/fast-livo2-demo/patches/`), applicate con `git am`:
  0001 VIO senza attesa del buffer lidar, 0002 porting ROS 2 + profilo Mid-360/D415,
  0003 thread OMP dall'affinita' della CPU, 0004 formattazione, 0005 correzioni di
  concorrenza nel nodo ROS 2.
- **Modifiche C2-Explorer (ottobre 2026)**: cartella dei log decisa all'avvio
  (`cartella_livo()` in `include/common_lib.h`, `$C2X_LIVO_DIR` o `/tmp/c2x_fast_livo2/`)
  invece della cartella dei sorgenti fissata in compilazione. In `src/LIVMapper.cpp`:
  - l'odometria porta l'istante della misura (l'ultimo aggiornamento), non quello di
    pubblicazione, come FAST-LIO: il ponte verso PX4 ne ricava la velocita';
  - con la telecamera (modo LIVO) l'odometria si pubblica dopo l'aggiornamento visivo
    (`pubblica_stato()` in `handleVIO()`), non tra il LIO e il VIO dello stesso istante;
  - iscrizione alla nuvola con la QoS dei sensori (best effort), come FAST-LIO;
  - nuvola colorata, immagine di debug, percorso e posa "mavros" solo con qualcuno in
    ascolto; ciclo principale a 1000 Hz invece di 5000 (meno CPU);
  - stdout in `/dev/null` di serie (`src/main.cpp`; `C2X_LIVO_STDOUT=1` per la diagnosi): le
    stampe a ogni fotogramma passavano per la pipe di ros2 launch e potevano bloccare il calcolo;
  - coda delle immagini di 2 (era 200000): se il calcolo resta indietro si salta un'immagine
    invece di accumulare ritardo; tempi per fotogramma (`ENABLE_PERFRAME_TIMING`) accesi di serie;
  - ritardo limitato: a ogni ciclo si riparte dall'immagine piu' recente gia' coperta da lidar e
    IMU, scartando le piu' vecchie (`sync_packages`, "immagini saltate per restare in tempo");
  - ciclo principale con un esecutore solo e `spin_all` (tutti i messaggi in attesa a ogni giro)
    invece di `spin_some` (uno per topic): l'IMU a 200 Hz restava in coda e il ritardo cresceva
    fino alla divergenza;
  - un solo `TransformBroadcaster` (prima uno nuovo a ogni pubblicazione);
  - `/lio_salute` come in FAST-LIO (punti, punti agganciati a un piano, l1), letto da
    `lio_to_px4_bridge` per l'atterraggio su stima degradata.
  In `CMakeLists.txt`: `Sophus_INCLUDE_DIRS` dal target `Sophus::Sophus`, `std_msgs`.
  Configurazione per il drone simulato: `config/c2x_sim.yaml`, `config/camera_c2x_sim.yaml`.
- **Dipendenze nel workspace** (`src/terze_parti/`): Sophus 1.22.10, rpg_vikit ROS 2
  (Robotic-Developer-Road, commit 4b7abc8, timeout di wait_for_service 100 ms -> 5 s come
  nello script di Intel; in `vikit_common/CMakeLists.txt` tolte le opzioni solo x86 su
  aarch64, C++17, configurazione CMake installata con percorsi relativi al posto di quella
  che puntava alla cartella di compilazione e si registrava in `~/.cmake/packages`), i soli
  messaggi di livox_ros_driver2 (MIT; dal 19 ottobre 2026 il driver completo, con Livox-SDK2
  dentro il progetto, vedi scripts/collaudo/LEGGIMI_COLLAUDO.md). In Sophus tolto `export(PACKAGE)`.
