#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
model=${1:-"$root/../rpo_description"}
expected=8e0f8f78903dc147a5c094c0fdc14b4a78f1b667
if [[ ! -d "$model" ]]; then
  git clone https://github.com/Roboparty/rpo_description.git "$model"
  git -C "$model" checkout "$expected"
fi
actual=$(git -C "$model" rev-parse HEAD)
if [[ "$actual" != "$expected" ]]; then
  echo "Model revision mismatch: $actual. Select a separate checkout at $expected." >&2
  exit 1
fi
echo "RPO model: $model ($actual)"
