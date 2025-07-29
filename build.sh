#!/bin/bash
set -euo pipefail

# -------------------------
# CONFIGURATION
# -------------------------

IMAGE_NAME="ros2"
TAG_SUFFIX=$(date +"%Y%m%d_%H%M")   # timestamp-based version
MAIN_TAG="${IMAGE_NAME}:${TAG_SUFFIX}"  # hfp_ssm:20250425_1423
ALT_TAG="${IMAGE_NAME}:h"    # hfp_ssm:h

DOCKERFILE="Dockerfile"
BUILD_CONTEXT="."

# Optional: Set to 1 to disable cache
NO_CACHE=0

# -------------------------
# BUILD EXECUTION
# -------------------------

echo "🚀 Building Docker image: ${MAIN_TAG} and ${ALT_TAG}"

export DOCKER_BUILDKIT=1

docker build \
  --file "${DOCKERFILE}" \
  --build-arg UID="$(id -u)" \
  --build-arg GID="$(id -g)" \
  $( [[ "${NO_CACHE}" == 1 ]] && echo "--no-cache" ) \
  --tag "${MAIN_TAG}" \
  "${BUILD_CONTEXT}"

# Add the extra tag
docker tag "${MAIN_TAG}" "${ALT_TAG}"

echo "✅ Build complete:"
echo "   - ${MAIN_TAG}"
echo "   - ${ALT_TAG}"