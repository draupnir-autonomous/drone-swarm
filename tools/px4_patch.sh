#!/usr/bin/env bash
# Modifiche locali a PX4-Autopilot <-> patches/px4/px4-locale.patch
#
# PX4 (6+ GB con la build) non sta nel repo: si clona il tag ufficiale (patches/px4/VERSIONE)
# e ci si applicano le modifiche del progetto (airframe 22000_gz_x500_mid360, gz_bridge, ...).
#
# Uso:
#   tools/px4_patch.sh salva     PX4-Autopilot -> patch   (dopo aver modificato file di PX4)
#   tools/px4_patch.sh applica   patch -> PX4-Autopilot   (dopo un git pull o su una macchina nuova)
#   tools/px4_patch.sh stato     dice se PX4 e patch coincidono   [--breve: una riga]
#
# Oltre alla patch: nel sottomodulo Gazebo di PX4 (Tools/simulation/gz) ci sono collegamenti al
# modello x500_mid360 e ai mondi del progetto (ros2_ws/...). Senza il modello PX4 non fa partire
# il drone. "salva" li elenca in patches/px4/collegamenti.txt, "applica" li ricrea.
# Altre modifiche dentro i sottomoduli di PX4 NON sono coperte (oggi non ce ne sono).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="$ROOT/PX4-Autopilot"
DIR="$ROOT/patches/px4"
PATCH="$DIR/px4-locale.patch"
LINK="$DIR/collegamenti.txt"
GZ_SUB="Tools/simulation/gz"
TAG="$(cat "$DIR/VERSIONE" 2>/dev/null || echo v1.17.0)"

# Collegamenti dal sottomodulo gz di PX4 verso il progetto: "<percorso in PX4> <destinazione>"
collegamenti_attuali() (
  cd "$PX4"
  find "$GZ_SUB" -type l -printf '%p %l\n' 2>/dev/null | while read -r p l; do
    case "$(cd "$(dirname "$p")" && realpath -m "$l")" in "$ROOT"/ros2_ws/*) echo "$p $l" ;; esac
  done | sort
)

# Ricrea i collegamenti che mancano; stampa quanti ne ha creati.
ricrea_collegamenti() {
  [ -f "$LINK" ] || return 0
  local n=0 p l
  while read -r p l; do
    [ -n "$p" ] || continue
    if [ ! -e "$PX4/$p" ] && [ ! -L "$PX4/$p" ]; then
      mkdir -p "$(dirname "$PX4/$p")"; ln -s "$l" "$PX4/$p"; n=$((n + 1))
    fi
  done < "$LINK"
  [ "$n" -gt 0 ] && echo "PX4: ricreati $n collegamenti a modello/mondi del progetto in $GZ_SUB."
  return 0
}

collegamenti_mancanti() {
  [ -f "$LINK" ] || { echo 0; return; }
  local n=0 p l
  while read -r p l; do [ -n "$p" ] && [ ! -e "$PX4/$p" ] && n=$((n + 1)); done < "$LINK"
  echo "$n"
}

# Diff completo di PX4 rispetto a HEAD: file modificati + file nuovi non ignorati.
diff_attuale() (
  cd "$PX4"
  git -c core.quotepath=off diff --binary --ignore-submodules=all HEAD
  # (il sottomodulo gz e' escluso: i suoi collegamenti al progetto stanno in collegamenti.txt)
  git -c core.quotepath=off ls-files --others --exclude-standard -z -- . ":(exclude)$GZ_SUB" \
    | while IFS= read -r -d '' f; do
        # --no-index esce con 1 quando i file sono diversi: e' il caso normale
        git diff --binary --no-index /dev/null "$f" || true
      done
)

controlla_px4() {
  if [ ! -d "$PX4/.git" ]; then
    echo "PX4-Autopilot non c'e': eseguire tools/setup_macchina.sh" >&2
    exit 1
  fi
  local head_tag
  head_tag="$(git -C "$PX4" describe --tags --exact-match HEAD 2>/dev/null || true)"
  if [ "$head_tag" != "$TAG" ]; then
    echo "ATTENZIONE: PX4 e' a '${head_tag:-$(git -C "$PX4" rev-parse --short HEAD)}', la patch e' per $TAG" >&2
  fi
}

cmd="${1:-stato}"
case "$cmd" in
  salva)
    controlla_px4
    mkdir -p "$DIR"
    tmp="$(mktemp)"; diff_attuale > "$tmp"
    if [ -f "$PATCH" ] && cmp -s "$tmp" "$PATCH"; then
      echo "PX4: patch invariata ($(grep -c '^diff --git' "$PATCH") file)."
      rm -f "$tmp"
    else
      mv "$tmp" "$PATCH"
      echo "PX4: patch aggiornata -> patches/px4/px4-locale.patch ($(grep -c '^diff --git' "$PATCH") file):"
      grep '^diff --git' "$PATCH" | sed 's|^diff --git a/[^ ]* b/|  |'
    fi
    tmp="$(mktemp)"; collegamenti_attuali > "$tmp"
    if [ -f "$LINK" ] && cmp -s "$tmp" "$LINK"; then
      rm -f "$tmp"
    else
      mv "$tmp" "$LINK"
      echo "PX4: collegamenti a modello/mondi aggiornati -> patches/px4/collegamenti.txt ($(wc -l < "$LINK"))."
    fi
    ;;

  applica)
    controlla_px4
    [ -f "$PATCH" ] || { echo "Nessuna patch in $PATCH"; exit 1; }
    tmp="$(mktemp)"; diff_attuale > "$tmp"
    if cmp -s "$tmp" "$PATCH"; then
      echo "PX4: gia' allineato alla patch."; rm -f "$tmp"; ricrea_collegamenti; exit 0
    fi
    cd "$PX4"
    if [ -s "$tmp" ]; then
      # Modifiche presenti in PX4 ma diverse dalla patch: copia di sicurezza, poi si tolgono.
      mkdir -p "$DIR/backup"
      bk="$DIR/backup/px4-prima-di-applica_$(date +%Y%m%d_%H%M%S).patch"
      cp "$tmp" "$bk"
      echo "PX4 aveva modifiche diverse dalla patch: copiate in ${bk#"$ROOT"/}"
      git apply -R --whitespace=nowarn "$tmp"
    fi
    rm -f "$tmp"
    git apply --whitespace=nowarn "$PATCH"
    echo "PX4: patch applicata ($(grep -c '^diff --git' "$PATCH") file). Ricompilare PX4: cd PX4-Autopilot && make px4_sitl_default"
    ricrea_collegamenti
    ;;

  stato)
    breve="${2:-}"
    if [ ! -d "$PX4/.git" ]; then
      echo "PX4: assente su questa macchina (tools/setup_macchina.sh lo clona e applica la patch)."; exit 0
    fi
    tmp="$(mktemp)"; diff_attuale > "$tmp" 2>/dev/null
    if [ -f "$PATCH" ] && cmp -s "$tmp" "$PATCH"; then
      echo "PX4: allineato alla patch."
    elif [ "$breve" = "--breve" ]; then
      echo "PX4: DIVERSO dalla patch (modificato qui -> tools/px4_patch.sh salva; patch nuova dal pull -> tools/px4_patch.sh applica)."
    else
      echo "PX4: diverso dalla patch."
      echo "  file modificati in PX4-Autopilot:"; grep '^diff --git' "$tmp" | sed 's|^diff --git a/[^ ]* b/|    |' || true
      echo "  file nella patch:";                  grep '^diff --git' "$PATCH" 2>/dev/null | sed 's|^diff --git a/[^ ]* b/|    |' || true
      echo "  Se hai modificato PX4 qui: tools/px4_patch.sh salva. Se la patch arriva da un pull: tools/px4_patch.sh applica."
    fi
    rm -f "$tmp"
    m="$(collegamenti_mancanti)"
    [ "$m" -gt 0 ] && echo "PX4: mancano $m collegamenti a modello/mondi del progetto (senza, il drone non parte) -> tools/px4_patch.sh applica"
    exit 0
    ;;

  *)
    sed -n '2,17p' "$0"; exit 1 ;;
esac
