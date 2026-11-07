---
name: riprendi
description: Riprende il lavoro all'inizio di una sessione, dopo un cambio di account Claude o di computer (Jetson, PC A, PC B). Sincronizza il repo in sicurezza, legge consegne e memoria, controlla patch PX4 e build di questa macchina, riassume dove eravamo e propone il prossimo passo. Da usare quando l'utente dice "riprendi", "dove eravamo", "ricominciamo", o apre il progetto su un'altra macchina.
---

# Ripresa del lavoro

## 1. Sincronizza (in sicurezza)
```bash
bash tools/claude/inizio_lavoro.sh
```
Lo script rifiuta il `git pull` se c'è una simulazione in corso o se ci sono modifiche non salvate,
fa solo fast-forward, segnala se la patch PX4 è cambiata e se la build di questa macchina manca o è
vecchia. Leggi tutto l'output. Se si ferma per modifiche locali: mostra all'utente quali file sono e
chiedi cosa fare (commit, stash o scartare): non scegliere tu. Se i rami sono divergenti, non fare
merge o rebase da solo: spiega la situazione e chiedi.

## 2. Ricostruisci il contesto
- `.claude/CONSEGNE.md` è già nel contesto (importato da CLAUDE.md), ma rileggilo se il pull l'ha
  cambiato.
- Leggi `.claude/memoria/MEMORY.md` e i file di argomento che riguardano i prossimi passi.
- `git log --oneline -10` e, se servono, i diff degli ultimi commit.
- `CLAUDE.local.md`: su che macchina sei e cosa c'è di diverso qui (architettura, RAM, GPU).
- Se le consegne citano run o file che stanno solo su un'altra macchina, tienine conto.

## 3. Controlla questa macchina
Se lo script segnala build mancante o vecchia, o patch PX4 da applicare, dillo e proponi i comandi
(`tools/px4_patch.sh applica`, `tools/setup_macchina.sh --compila`). Non avviare build lunghe
(20+ minuti) o simulazioni senza chiedere.

## 4. Riassumi e proponi
Al massimo 10 righe: obiettivo, ultimo stato con i numeri chiave, cosa è cambiato dall'ultima volta,
eventuali problemi della macchina, e il primo dei prossimi passi come proposta concreta. Poi aspetta
la conferma dell'utente.
