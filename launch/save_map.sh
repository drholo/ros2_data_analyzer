#!/bin/bash
# Wrapper to save map with retry logic
# Usage: save_map.sh <output_path>

OUTPUT_PATH="$1"
RETRIES=10
DELAY=1

for ((i=0; i<RETRIES; i++)); do
    sleep "$DELAY"
    if ros2 run nav2_map_server map_saver_cli --mode raw -f "$OUTPUT_PATH"; then
        exit 0
    fi
    echo "Map save attempt $((i+1)) failed, retrying..."
done

echo "Failed to save map after $RETRIES attempts"
exit 1
