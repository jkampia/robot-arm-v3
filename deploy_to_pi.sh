#!/usr/bin/env bash
set -euo pipefail

# Edit these for your Raspberry Pi.
REMOTE_USER="jonathan"
REMOTE_HOST="10.0.0.175"
REMOTE_PARENT_DIR="/home/${REMOTE_USER}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_NAME="$(basename "${REPO_DIR}")"

echo "Pushing ${REPO_NAME} to ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PARENT_DIR}/"

rsync -az --delete \
  --exclude ".git/" \
  --exclude "venv/" \
  --exclude "__pycache__/" \
  --exclude "*.pyc" \
  --exclude "src/chessarm/ui/frontend/node_modules/" \
  "${REPO_DIR}" \
  "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PARENT_DIR}/"

echo "Done: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PARENT_DIR}/${REPO_NAME}"
