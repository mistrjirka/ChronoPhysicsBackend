#!/bin/bash

# Output file to store library versions
OUTPUT_FILE="library_versions.txt"

# Path to the compiled binary (adjust if needed)
BINARY_PATH="build/main"

# Check if the binary exists
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Binary file not found at $BINARY_PATH"
    exit 1
fi

# Start capturing library versions
echo "Capturing library versions for $BINARY_PATH..." > "$OUTPUT_FILE"

# List shared libraries used by the binary
ldd "$BINARY_PATH" | awk '{print $1}' | while read -r LIBRARY; do
    if pacman -Q "$LIBRARY" &> /dev/null; then
        # Get the version of the library from pacman
        VERSION=$(pacman -Q "$LIBRARY")
        echo "$VERSION" >> "$OUTPUT_FILE"
    else
        # Attempt to extract version from the library file itself
        LIBRARY_PATH=$(ldconfig -p | grep "$LIBRARY" | awk '{print $NF}' | head -n 1)
        if [ -f "$LIBRARY_PATH" ]; then
            VERSION=$(strings "$LIBRARY_PATH" | grep -i 'version' | head -n 1)
            echo "$LIBRARY ($VERSION)" >> "$OUTPUT_FILE"
        else
            echo "$LIBRARY: Not found in pacman or ldconfig" >> "$OUTPUT_FILE"
        fi
    fi
done

echo "Library versions saved to $OUTPUT_FILE"