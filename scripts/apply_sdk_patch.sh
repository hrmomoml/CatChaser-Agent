#!/usr/bin/env bash
set -euo pipefail
repo=${1:?Usage: apply_sdk_patch.sh /path/to/roboparty_deploy}
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
actual=$(git -C "$repo/src/inference" rev-parse HEAD)
expected=798f254a1754daff0a964a0e2db58e1c5a5450f7
if [[ "$actual" != "$expected" ]]; then
  echo "Inference revision mismatch: $actual (expected $expected)" >&2
  exit 1
fi
if git -C "$repo/src/inference" apply --reverse --check "$root/patches/rpo_external_arms.patch" 2>/dev/null; then
  echo "Patch already applied"
  exit 0
fi
git -C "$repo/src/inference" apply --check "$root/patches/rpo_external_arms.patch"
git -C "$repo/src/inference" apply "$root/patches/rpo_external_arms.patch"
echo "Applied. Rebuild the official workspace on the robot; no hardware was initialized."
