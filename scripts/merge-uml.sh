#!/usr/bin/env bash
set -euo pipefail

# Assemble src/uml/uml.mmd from per-class diagrams and relationships.mmd
# Portable equivalent of scripts/merge-uml.ps1

# Find repo root: parent of script dir that contains build.gradle
script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$script_dir/.."
while [ ! -f "$repo_root/build.gradle" ]; do
  if [ "$repo_root" = "/" ] || [ -z "$repo_root" ]; then
    echo "Could not find project root (build.gradle) above $script_dir" >&2
    exit 2
  fi
  repo_root="$(dirname "$repo_root")"
done
repo_root="$(cd "$repo_root" && pwd)"

uml_root="$repo_root/src/uml"
out_file="$uml_root/uml.mmd"
rel_file="$uml_root/relationships.mmd"

# Helper: extract class block from file content using perl for portability
extract_class_block() {
  perl -0777 -ne 'if (/(?ms)^[ \t]*class[ \t]+[A-Za-z_][A-Za-z0-9_]*[ \t]*\{.*?^[ \t]*\}\s*$/) { print $& }' -- "$1"
}

chunks_tmp="$(mktemp)"
rel_lines_tmp="$(mktemp)"
cleanup() { rm -f "$chunks_tmp" "$rel_lines_tmp" || true; }
trap cleanup EXIT

# Collect files under frc and edu using find to be robust
frc_files=()
edu_files=()
if [ -d "$uml_root/frc" ]; then
  while IFS= read -r -d $'\0' f; do
    frc_files+=("$f")
  done < <(find "$uml_root/frc" -type f -name '*.mmd' -print0 2>/dev/null || true)
fi
if [ -d "$uml_root/edu" ]; then
  while IFS= read -r -d $'\0' f; do
    edu_files+=("$f")
  done < <(find "$uml_root/edu" -type f -name '*.mmd' -print0 2>/dev/null || true)
fi

class_files=("${frc_files[@]}" "${edu_files[@]}")

# Sort files by a lowercase normalized key for deterministic, case-insensitive ordering
if [ ${#class_files[@]} -gt 0 ]; then
  IFS=$'\n' sorted=( $(printf "%s\n" "${class_files[@]}" | \
    while IFS= read -r f; do printf "%s\t%s\n" "$(printf "%s" "$f" | tr '[:upper:]' '[:lower:]')" "$f"; done | sort | cut -f2) )
else
  sorted=()
fi

count=0
for f in "${sorted[@]}"; do
  [ -f "$f" ] || continue
  block=$(extract_class_block "$f" | sed 's/\r$//' ) || true
  if [ -z "$block" ]; then
    echo "Warning: No class block in $f" >&2
    continue
  fi
  rel=${f#${uml_root}/}
  rel=${rel//\\//}
  printf "  %%%% --- %s ---\n" "$rel" >> "$chunks_tmp"
  printf "%s\n" "$block" >> "$chunks_tmp"
  printf "\n" >> "$chunks_tmp"
  count=$((count+1))
done

if [ -f "$rel_file" ]; then
  sed 's/\r$//' "$rel_file" > "$rel_lines_tmp"
else
  : > "$rel_lines_tmp"
fi

# Build output
{
  printf "classDiagram\n"
  printf "  %%%% AUTO-GENERATED - do not edit by hand. Run: gradlew mergeUml\n"
  printf "  %%%% LR layout reduces vertical crossing; associations below are simplified (see class members for full deps)\n"
  printf "  direction LR\n\n"
  cat "$chunks_tmp"
  cat "$rel_lines_tmp"
} > "$out_file"

# Ensure final newline
if [ -s "$out_file" ]; then
  perl -0777 -pe 's/\s+\z/\n/' -i "$out_file" || true
fi

echo "Wrote $out_file ($count class files + relationships)"