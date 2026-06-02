#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

if ! command -v npm >/dev/null 2>&1; then
  echo "npm is not installed or is not on PATH."
  exit 1
fi

if ! command -v node >/dev/null 2>&1; then
  echo "node is not installed or is not on PATH."
  exit 1
fi

NODE_MAJOR="$(node -p "process.versions.node.split('.')[0]")"
if [ "$NODE_MAJOR" -lt 18 ]; then
  echo "Node $(node --version) is too old for this frontend."
  echo "Install Node 18 or newer, then rerun this script."
  exit 1
fi

if [ ! -d "node_modules" ]; then
  echo "Frontend dependencies are not installed."
  echo "Run: npm install"
  exit 1
fi

exec npm run dev -- --host 0.0.0.0
