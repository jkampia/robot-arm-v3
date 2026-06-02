#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PYTHON="$PROJECT_ROOT/venv/bin/python"

if [ ! -x "$PYTHON" ]; then
  PYTHON="python3"
fi

cd "$SCRIPT_DIR"

if ! "$PYTHON" -c "import uvicorn" >/dev/null 2>&1; then
  echo "uvicorn is not installed for $PYTHON."
  echo "Run: $PYTHON -m pip install -r $PROJECT_ROOT/requirements.txt"
  exit 1
fi

exec "$PYTHON" -m uvicorn main:app --host 0.0.0.0 --port 8080