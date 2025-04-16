#!/bin/bash

# Set the library path to include the local libs directory
export LD_LIBRARY_PATH="$(pwd)/libs:$LD_LIBRARY_PATH"

# Path to the binary
BINARY_PATH="build/main"

# Check if the binary exists
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Binary file not found at $BINARY_PATH"
    exit 1
fi

# Run the binary
"$BINARY_PATH" "$@"
