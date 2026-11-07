---
name: portabilita-macchine-account
description: Il progetto passa tra 3 macchine (Jetson principale, PC A, PC B) e 2 account Claude; come viaggia il contesto e cosa non viaggia
metadata:
  type: project
---

Dal 7 nov 2026 il progetto è un repo git (remoto GitHub privato) e si lavora a turno su:
- **Jetson Orin** (principale, aarch64, Ubuntu 24.04, ROS2 Jazzy, 7 GB RAM + 8 GB swap, 6 core);
- **PC A** e **PC B** (Ubuntu 24.04, x86_64);
con due account Claude che si alternano. Il contesto di Claude viaggia SOLO attraverso il repo:
`CLAUDE.md` (stabile), `.claude/CONSEGNE.md` (stato del lavoro, riscritto a ogni /consegne),
questa cartella `.claude/memoria/` (memoria automatica, spostata qui con `autoMemoryDirectory` in
`.claude/settings.local.json`, generato per macchina da `tools/claude/configura_claude.sh`).

Non viaggia: le conversazioni (restano nella macchina dove sono nate), le build
(`ros2_ws/build|install|log`, `PX4-Autopilot/build`: architetture diverse), PX4 stesso (si clona
v1.17.0 e si applica `patches/px4/`), i dati pesanti di `risultati_prova/` (in git solo txt/md/json/yaml/csv).

ARM↔x86 (7 nov): build mai condivise (anche `-march=native`); dipendenze comuni a entrambe in
`setup_macchina.sh --dipendenze` (rosdep del workspace, python3-scipy, Open3D 0.20.0 via pip --user,
che prima non installava nessuno script); i 6 collegamenti PX4 Tools/simulation/gz -> modello
x500_mid360 e mondi stanno in patches/px4/collegamenti.txt (senza il modello il drone non parte).
Non ancora provato su un PC x86 reale.

**Why:** l'utente vuole cambiare account o computer a fine lavoro e ripartire con tutto il contesto.
**How to apply:** a fine lavoro usare /consegne (aggiorna CONSEGNE.md, salva patch PX4, commit, push);
all'inizio /riprendi. Mai `git pull` con una run in corso (bash legge gli script mentre girano). Una run
lunga lanciata su una macchina NON si sposta: nelle consegne va scritto dove sta girando.
Vedi [[swarm-20261025]] per lo stato tecnico.
