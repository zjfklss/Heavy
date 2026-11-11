#!/usr/bin/env bash
set -euo pipefail
sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$sim_root/scripts/start_simulation.sh" "$@"
