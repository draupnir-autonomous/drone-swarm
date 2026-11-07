#!/usr/bin/env bash
# Fine lavoro: salva la patch PX4, committa tutto e fa push su GitHub.
# Lo chiama /consegne dopo aver aggiornato .claude/CONSEGNE.md; si puo' usare anche a mano.
#
# Uso: tools/claude/fine_lavoro.sh "riassunto in una riga"
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/comune.sh"
cd "$ROOT" || exit 1

MSG="${1:-lavoro del $(date '+%d/%m %H:%M')}"
MACCHINA="$(nome_macchina)"
LIMITE_KB=$((50 * 1024))   # GitHub rifiuta file > 100 MB e avvisa sopra i 50 MB

git rev-parse --git-dir >/dev/null 2>&1 || { echo "Non e' un repo git: vedi tools/PORTABILITA.md"; exit 1; }

# 1. Patch PX4 (PX4 non e' nel repo: viaggiano solo le sue modifiche)
if [ -d PX4-Autopilot/.git ]; then
  bash tools/px4_patch.sh salva || echo "ATTENZIONE: patch PX4 non salvata."
fi

# 2. Tutto in stage, tranne i file troppo grossi per GitHub
git add -A
grossi="$(git diff --cached --name-only --diff-filter=AM -z | xargs -0 -r du -k 2>/dev/null | awk -F'\t' -v l=$LIMITE_KB '$1>l {print $2}')"
if [ -n "$grossi" ]; then
  echo "File oltre 50 MB tolti dal commit (aggiungili a .gitignore o spostali in risultati_prova/):"
  while IFS= read -r f; do echo "  $f"; git reset -q -- "$f"; done <<< "$grossi"
fi

# 3. Commit (con [macchina] davanti: l'hook di avvio lo usa per capire da dove arriva il lavoro)
if git diff --cached --quiet; then
  echo "Niente di nuovo da committare."
else
  trailer=""
  [ -n "${CLAUDECODE:-}" ] && trailer=$'\n\nCo-Authored-By: Claude <noreply@anthropic.com>'
  git commit -q -m "[$MACCHINA] $MSG$trailer" && echo "Commit: $(git log -1 --oneline)"
fi

# 4. Push
if ! git remote get-url origin >/dev/null 2>&1; then
  echo "Nessun remoto 'origin': il commit resta solo su questa macchina (vedi tools/PORTABILITA.md)."
  exit 0
fi
attive="$(run_attive)"
[ -n "$attive" ] && echo "Nota: c'e' una simulazione in corso qui; il push non la disturba, ma scrivila nelle consegne."
if git_muto push -u origin HEAD 2>&1 | tail -3; then
  if [ "$(git rev-list --count '@{u}..HEAD' 2>/dev/null || echo 1)" = "0" ]; then
    echo "Push riuscito: $(git rev-parse --abbrev-ref '@{u}') e' aggiornato."
    exit 0
  fi
fi
echo "PUSH NON RIUSCITO. Il commit e' salvato in locale. Cause tipiche:"
echo "  - niente rete o credenziali GitHub mancanti: sistemare e poi  git push"
echo "  - su GitHub ci sono commit fatti da un'altra macchina (\"rejected\", \"fetch first\"):"
echo "    git pull --rebase  e poi  git push   (se ci sono conflitti, fatti aiutare da Claude)"
exit 2
