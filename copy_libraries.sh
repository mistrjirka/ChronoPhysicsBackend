#!/bin/bash

# Path to the compiled binary (adjust if needed)
BINARY_PATH="build/main"

# Directory to store the copied libraries
LIBS_DIR="libs"

# Check if the binary exists
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Binary file not found at $BINARY_PATH"
    exit 1
fi

# Create the libs directory if it doesn't exist
mkdir -p "$LIBS_DIR"

# List shared libraries used by the binary and copy them
ldd "$BINARY_PATH" | awk '{print $3}' | while read -r LIBRARY_PATH; do
    if [ -f "$LIBRARY_PATH" ]; then
        cp -u "$LIBRARY_PATH" "$LIBS_DIR/"
        echo "Copied: $LIBRARY_PATH"
    else
        echo "Skipping: $LIBRARY_PATH (not a file)"
    fi
done

echo "All libraries copied to $LIBS_DIR."

echo "To run the binary with these libraries, use:"
echo "export LD_LIBRARY_PATH=./libs:$LD_LIBRARY_PATH"
echo "./main"