#!/bin/bash
# Compatibility wrapper for the nrf54-motion USB flash entrypoint.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Note: flash.sh is kept as a compatibility wrapper."
echo "      Prefer ./build_and_flash.sh for explicit USB flashing."

exec "$SCRIPT_DIR/build_and_flash.sh" "$@"
