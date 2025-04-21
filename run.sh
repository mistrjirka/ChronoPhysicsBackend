#!/bin/bash

# Build the Docker image
docker build -t myapp -f Dockerfile.build .

# Run the container with port mapping and pass arguments
docker run -it --rm \
  -p 17863:17863 \
  -p 9090:9090 \
  --network=host \
  myapp "$@"