# Lavorare su Jetson, PC A e PC B con due account Claude

La **Jetson** è la macchina principale. Il progetto passa da una macchina all'altra attraverso un
repo **GitHub privato**, e il contesto di Claude viaggia **dentro il repo**: qualsiasi account, su
qualsiasi delle tre macchine, apre il progetto e riparte da dove eravate rimasti.

## Cosa viaggia e cosa no

| viaggia (in git) | resta sulla macchina |
|---|---|
| `CLAUDE.md`: istruzioni stabili per Claude | le conversazioni con Claude |
| `.claude/CONSEGNE.md`: dove eravamo, prossimi passi | `CLAUDE.local.md`: scheda della macchina |
| `.claude/memoria/`: memoria automatica di Claude | `.claude/settings.local.json`: puntatore alla memoria |
| `.claude/skills/`: comandi `/consegne` e `/riprendi` | build: `ros2_ws/build install log`, `PX4-Autopilot/build` |
| sorgenti di `ros2_ws`, script, mondi, LEGGIMI | PX4-Autopilot e l'agente DDS (li riscarica `setup_macchina.sh`) |
| `patches/px4/`: le modifiche a PX4 v1.17.0 | dati pesanti delle prove (ulg, ply, pcd, png, log) |
| riassunti delle prove (txt, md, json, yaml, csv) | le simulazioni in corso |

Una conversazione non si sposta tra macchine, ma non serve spostarla: a fine lavoro Claude scrive
tutto quello che conta in `CONSEGNE.md` e nella memoria, e la sessione successiva lo legge.

---

## Una volta sola

### 1. Jetson: collegare GitHub e fare il primo push
```bash
ssh-keygen -t ed25519 -C "jetson"      # Invio a tutte le domande
cat ~/.ssh/id_ed25519.pub              # da copiare su GitHub
```
Su github.com: **Settings → SSH and GPG keys → New SSH key**, incolla la chiave. Poi crea un
repository **privato** vuoto (senza README), per esempio `drone-swarm`, e:
```bash
cd ~/Documents/Drone_20261025_SWARM
ssh -T git@github.com                  # la prima volta rispondi "yes"
git remote add origin git@github.com:<tuo-utente>/drone-swarm.git
tools/claude/fine_lavoro.sh "primo caricamento"
```

### 2. PC A e PC B (Ubuntu 24.04)
Su ognuno: installa Claude Code (app desktop e/o CLI), crea una chiave SSH e aggiungila a GitHub
come sopra, poi:
```bash
git clone git@github.com:<tuo-utente>/drone-swarm.git ~/Documents/Drone_SWARM
cd ~/Documents/Drone_SWARM
tools/setup_macchina.sh --dipendenze --compila --nome pc-a   # pc-b sull'altro
```
Il percorso può essere diverso su ogni macchina, anche con un utente Linux diverso: gli script
usano solo percorsi relativi. La prima volta lo script scarica PX4 (alcuni GB), installa ROS2 Jazzy
e Gazebo (chiede `sudo`) e compila tutto: conta un'ora o più. La build della Jetson (aarch64)
non vale sul PC (x86_64) e viceversa.

---

## Ogni volta

### Finisci di lavorare
Nella sessione di Claude scrivi **`/consegne`**. Claude riscrive `.claude/CONSEGNE.md`, aggiorna la
memoria, salva le modifiche a PX4 in `patches/px4/`, fa commit e push. Se lasci una campagna in
esecuzione, Claude lo scrive nelle consegne (la simulazione resta su quella macchina).

### Cambi account sulla stessa macchina
- **App desktop**: cambia account dal menu account dell'app, apri la cartella del progetto,
  nuova sessione, scrivi **`/riprendi`**.
- **CLI**: `tools/claude/cambia_account.sh` (fa `claude auth logout` e `claude auth login`), poi
  `claude` nella cartella del progetto e **`/riprendi`**. In alternativa, solo sulla stessa macchina,
  `claude --continue` riprende la conversazione di prima così com'era. Se la conversazione è lunga,
  però, la prima risposta consuma molto del limite del nuovo account: in genere conviene `/riprendi`.

Memoria e consegne sono le stesse per entrambi gli account, perché stanno nel progetto: cambia solo
l'account che consuma i messaggi.

### Cambi computer
1. Sulla macchina che lasci: **`/consegne`** (commit e push).
2. Sulla macchina dove arrivi: apri il progetto con Claude e scrivi **`/riprendi`**. Claude fa il
   `git pull` (solo se non c'è una simulazione in corso e non ci sono modifiche locali), applica la
   patch PX4 se è cambiata, controlla le build e ti riassume dove eravate.

All'avvio di ogni sessione un hook dice a Claude su che macchina si trova, se il repo è indietro
rispetto a GitHub e se c'è una simulazione in corso. Se l'ultimo lavoro è stato fatto altrove,
Claude ti propone `/riprendi` da solo.

---

## Regole
- **Una macchina alla volta.** Se lavori su due macchine senza `/consegne` in mezzo, le storie
  divergono e va fatto `git pull --rebase` (Claude ti aiuta con eventuali conflitti).
- **Mai `git pull` con una simulazione in corso**: bash legge gli script mentre girano e un pull
  può uccidere la run. `inizio_lavoro.sh` si rifiuta da solo.
- Se modifichi PX4 a mano, fuori da Claude: `tools/px4_patch.sh salva` prima del commit (lo fa già
  `fine_lavoro.sh`).
- I dati pesanti delle prove restano dove sono stati prodotti, quasi sempre la Jetson. Se ti servono
  su un PC: `rsync -a <utente>@<ip-jetson>:Documents/Drone_20261025_SWARM/risultati_prova/<run> risultati_prova/`

## Script

| script | cosa fa |
|---|---|
| `tools/setup_macchina.sh` | prepara una macchina: PX4 + patch, agente DDS, dipendenze, build, Claude |
| `tools/claude/inizio_lavoro.sh` | pull sicuro e controlli (lo usa `/riprendi`) |
| `tools/claude/fine_lavoro.sh "msg"` | patch PX4, commit `[macchina] msg`, push (lo usa `/consegne`) |
| `tools/claude/cambia_account.sh` | cambia l'account della CLI |
| `tools/claude/configura_claude.sh` | memoria nel repo e scheda della macchina (rifallo se sposti la cartella) |
| `tools/px4_patch.sh salva / applica / stato` | modifiche locali a PX4 tra cartella e patch |

## Se qualcosa non va
- **Claude non ricorda niente**: controlla che esista `.claude/settings.local.json` con
  `autoMemoryDirectory`; se manca, `tools/claude/configura_claude.sh` e apri una sessione nuova.
- **Push rifiutato**: `git pull --rebase` e poi `git push`.
- **`git apply` fallisce su PX4**: PX4 non è al tag giusto (`git -C PX4-Autopilot describe --tags`
  deve dire `v1.17.0`). Le modifiche che avevi sono in `patches/px4/backup/`.
- I vecchi `.git` di Sophus, rpg_vikit, ikd-Tree e Livox-SDK2 sono in `_git_annidati_backup/` sulla
  Jetson: quelle librerie ora stanno nel repo con le modifiche già dentro (vedi `patches/terze_parti/`).
