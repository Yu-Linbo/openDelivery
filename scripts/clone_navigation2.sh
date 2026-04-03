#!/usr/bin/env bash
# Optional: vendor Navigation2 sources next to openDelivery for local patches (Foxy branch).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${ROOT}/src/navigation/navigation2"
BRANCH="${NAV2_BRANCH:-foxy-devel}"
if [[ -d "${DEST}/.git" ]]; then
  echo "Already cloned: ${DEST}"
  exit 0
fi
mkdir -p "$(dirname "${DEST}")"
git clone --depth 1 -b "${BRANCH}" https://github.com/ros-planning/navigation2.git "${DEST}"
echo "Cloned Navigation2 (${BRANCH}) -> ${DEST}"
echo "Add to workspace and: rosdep install --from-paths src/navigation/navigation2 -y"
