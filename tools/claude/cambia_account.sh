#!/usr/bin/env bash
# Cambia l'account Claude usato dalla CLI (`claude`) su questa macchina, poi indica come ripartire.
# L'app desktop ha un login suo: li' l'account si cambia dal menu account dell'app.
#
# Uso: tools/claude/cambia_account.sh [--avvia]
#   --avvia  dopo il cambio apre subito `claude` nel progetto e lancia /riprendi
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/comune.sh"
command -v claude >/dev/null || { echo "claude non e' installato su questa macchina."; exit 1; }

mostra_account() {
  claude auth status 2>/dev/null | python3 -I -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    print("  (stato non leggibile)"); sys.exit()
if not d.get("loggedIn"):
    print("  nessun account collegato")
else:
    print("  %s  (%s %s)" % (d.get("email", "?"), d.get("subscriptionType") or "", d.get("orgName") or ""))
'
}

if [ -n "${ANTHROPIC_API_KEY:-}" ]; then
  echo "ATTENZIONE: ANTHROPIC_API_KEY e' impostata: la CLI usa quella chiave e non l'account."
  echo "Toglila (unset ANTHROPIC_API_KEY, e da ~/.bashrc) se vuoi usare gli account claude.ai."
fi

echo "Account attuale della CLI:"; mostra_account
if [ -n "$(git -C "$ROOT" status --porcelain 2>/dev/null)" ]; then
  echo "Nota: nel progetto ci sono modifiche non committate. Il cambio account non le tocca,"
  echo "ma se cambi anche computer fai prima /consegne (o tools/claude/fine_lavoro.sh)."
fi
read -r -p "Esco da questo account e accedo con l'altro? [s/N] " r
[[ "$r" =~ ^[sS] ]] || { echo "Nessun cambio."; exit 0; }

claude auth logout || true
echo "Ora si apre il login (browser o link da aprire): entra con l'ALTRO account."
claude auth login
echo "Account adesso:"; mostra_account

cat <<EOF

Ripartire con tutto il contesto del progetto:
  cd "$ROOT" && claude          e poi scrivi /riprendi
Riprendere la stessa identica conversazione di prima (solo su questa macchina; una conversazione
lunga consuma molto del limite del nuovo account alla prima risposta):
  cd "$ROOT" && claude --continue
EOF
if [ "${1:-}" = "--avvia" ]; then
  cd "$ROOT" && exec claude "/riprendi"
fi
