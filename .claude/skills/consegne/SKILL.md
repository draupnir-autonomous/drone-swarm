---
name: consegne
description: Passaggio di consegne a fine lavoro. Riscrive .claude/CONSEGNE.md, aggiorna la memoria in .claude/memoria/, salva la patch PX4, fa commit e push, così un altro account Claude o un altro computer (Jetson, PC A, PC B) riparte esattamente da qui. Da usare quando l'utente ha finito, cambia account, cambia computer, dice "chiudo", "consegne", "handoff", "passo al PC", "passo alla Jetson".
---

# Passaggio di consegne

Obiettivo: chi apre il progetto dopo di te (altro account, altra macchina, sessione nuova) non ha
questa conversazione. Ha solo `CLAUDE.md`, `.claude/CONSEGNE.md`, `.claude/memoria/` e la storia git.
Tutto quello che serve per continuare deve finire lì.

## 1. Raccogli lo stato reale (non a memoria)
Esegui ed esamina:
```bash
git status --short | head -50
git diff --stat HEAD | tail -5
git log --oneline -5
pgrep -af "run_multi_exploration|campagna_sciame|gz sim|ros2 launch" | grep -v pgrep || true
ls -td risultati_prova/*/ | head -5
```
Se c'è una simulazione o una campagna in corso: NON fermarla. Scrivi nelle consegne su quale
macchina sta girando, cosa sta facendo, dove scrive i risultati e quando dovrebbe finire.

## 2. Riscrivi `.claude/CONSEGNE.md`
Riscrivilo da capo (non aggiungere in coda), mantenendo queste sezioni:
- intestazione: data e ora, macchina (`hostname`, `uname -m`), ultimo commit;
- **Obiettivo in corso**: cosa vuole ottenere l'utente, con le sue parole se utili;
- **Dove eravamo rimasti**: cosa è stato fatto in questa sessione, con numeri e percorsi delle run;
  cosa è stato provato e scartato e perché;
- **Prossimi passi**: lista numerata, il primo deve essere eseguibile subito;
- **Attenzione**: trappole attive (run da rifare, codice compilato a metà, parametri temporanei);
- **Stato della macchina**: processi in corso, file importanti che restano solo su questa macchina.

Massimo ~120 righe. Le cose durevoli (lezioni, decisioni, misure di riferimento) non vanno qui ma in
memoria o in `LEGGIMI.md`.

## 3. Aggiorna la memoria
In `.claude/memoria/` (è la memoria automatica di questo progetto): aggiorna i file di argomento
esistenti o creane uno nuovo, e la riga corrispondente in `MEMORY.md` (una riga per file, indice sotto
le 200 righe). Correggi o cancella le memorie diventate false. Se la sessione ha prodotto risultati
che l'utente vorrà rileggere, aggiorna anche la sezione giusta di `LEGGIMI.md`.

## 4. Commit e push
```bash
bash tools/claude/fine_lavoro.sh "<riassunto in una riga di cosa è cambiato>"
```
Lo script salva la patch PX4 (`tools/px4_patch.sh salva`), fa `git add -A`, commit e push. Leggi
l'output: se il push fallisce (rete, remoto mancante, rifiuto) dillo chiaramente all'utente con il
comando per riprovare. Non usare `--force`.

## 5. Riferisci all'utente
In poche righe: commit creato, push riuscito o no, cosa resta in esecuzione su questa macchina, e
come ripartire altrove: "sull'altra macchina o con l'altro account apri il progetto e scrivi `/riprendi`".
