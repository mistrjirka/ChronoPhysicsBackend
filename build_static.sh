#!/bin/bash
set -e

# Build a Docker image specifically for creating the static build
docker build -t chronophysics-builder -f Dockerfile.build .

# Create a container that builds the static binary
docker run --rm -v $(pwd):/output chronophysics-builder

echo "Static binary is available at: ./build_static/main"
echo "You can run it directly with: ./build_static/main [options]"
