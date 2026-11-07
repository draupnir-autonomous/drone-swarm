# Drone SWARM — istruzioni per Claude

Esplorazione autonoma multi-drone (port di C²-Explorer) su **ROS2 Jazzy + PX4 v1.17 + Gazebo
Harmonic**, Ubuntu 24.04, con volo senza GPS (FAST-LIO2). Si parla italiano.

Il progetto passa tra **tre macchine** e **due account Claude**:
- **Jetson Orin** — macchina principale (aarch64, 7 GB RAM + 8 GB swap, 6 core): simulazioni e drone vero;
- **PC A** e **PC B** — Ubuntu 24.04 x86_64.
Le note specifiche della macchina corrente sono in `CLAUDE.local.md` (non versionato, generato da
`tools/claude/configura_claude.sh`). La memoria automatica è in `.claude/memoria/` (versionata).

## Stato del lavoro (passaggio di consegne)

@.claude/CONSEGNE.md

## Ciclo di lavoro tra macchine e account
- **Inizio**: `/riprendi` — sincronizza (solo se nessuna run è in corso), legge consegne e memoria,
  controlla build e patch PX4, riassume e propone il passo successivo.
- **Fine** (o prima di cambiare account/computer): `/consegne` — riscrive `.claude/CONSEGNE.md`,
  aggiorna la memoria, salva la patch PX4, commit e push.
- Se l'utente dice "ho finito", "cambio account", "passo al PC": proponi `/consegne`.

## Dove sono le cose
| percorso | cosa |
|---|---|
| `LEGGIMI.md` | documentazione completa e storia degli esperimenti (100 KB: leggi solo le sezioni che servono) |
| `ros2_ws/src/c2x_px4_bridge/` | ponti verso PX4: odometria, FAST-LIO→EKF2, comandi, nuvole |
| `ros2_ws/src/exploration_manager/` | macchina a stati, allocazione fra droni (strisce, visita) |
| `ros2_ws/src/plan_env/` | mappe, scambio mappe fra droni (MultiMapManager) |
| `ros2_ws/scripts/run_multi_exploration.sh` | la missione su qualsiasi mondo, con N droni |
| `ros2_ws/scripts/campagna_sciame.sh`, `pagella_sciame.py` | campagne 1 vs N droni e pagella |
| `ros2_ws/scripts/collaudo/` | drone vero (Mid-360S, Pixhawk): procedure e registri |
| `patches/px4/` | modifiche locali a PX4 v1.17.0 (airframe 22000, gz_bridge) e collegamenti dal sottomodulo gz di PX4 al modello x500_mid360 e ai mondi |
| `tools/` | portabilità: `setup_macchina.sh`, `px4_patch.sh`, `claude/` |

I commenti nel codice riportano anche le modifiche provate e scartate, con i numeri: leggerli prima
di riproporre un esperimento.

## Comandi base
```bash
# build (da ros2_ws, ambiente ROS caricato)
colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF
# missione (da ros2_ws, PATH SENZA /opt/ros davanti: il gz ridotto di ROS non ha `gz sim`)
echo N | env PF_MIN_RAM_GB=3 MOSTRA_MAPPA=0 ODOM_SOURCE=lio ./scripts/run_multi_exploration.sh <mondo> <n_droni>
# campagna e pagella
RUN="office_small:3:1 office_small:1:1" setsid nohup ./scripts/campagna_sciame.sh &
python3 scripts/pagella_sciame.py ../risultati_prova/<run> [...]   # una riga per run
```

## Regole di portabilità
- Mai percorsi assoluti in codice e script: il progetto deve girare da qualsiasi cartella, utente e
  macchina (`scripts/percorsi_relativi.py` sistema i link di `install/` dopo ogni build).
- `build/`, `install/`, `log/` non si versionano: tra Jetson (aarch64) e PC (x86_64) si ricompila.
- PX4 non è nel repo: dopo aver modificato file in `PX4-Autopilot/` esegui `tools/px4_patch.sh salva`
  (lo fa anche `/consegne`). Su una macchina nuova `tools/setup_macchina.sh` clona e applica.
- `risultati_prova/`: in git solo i riassunti testuali (txt, md, json, yaml, csv). Se un'analisi
  serve dati pesanti (ulg, ply, pcd), dì su quale macchina si trovano.
- Prima di `git pull`: nessuna simulazione in corso (`pgrep -af "run_multi_exploration|gz sim"`).

## Lezioni da non dimenticare
- Bash legge gli script mentre girano: modificare `run_multi_exploration.sh` durante una run la uccide.
- Le run lunghe vanno staccate con `setsid nohup`: quelle lanciate dalla shell di Claude muoiono con la sessione.
- Sulla Jetson lo sciame gira a RTF basso: i tempi missione si misurano in secondi simulati (`TEMPO_SIM=1`).
- Sophus va compilato con test ed esempi spenti (falliscono con -Werror su aarch64).
- Non usare il `build.sh` di livox_ros_driver2: cancella build/ e install/ di tutto il workspace.
