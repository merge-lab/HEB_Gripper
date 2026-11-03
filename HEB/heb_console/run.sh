#!/usr/bin/env bash
# Launch HEB Console. Creates a local virtualenv with its dependencies on
# first run, so nothing is installed into the system Python.
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
    echo "First run: creating .venv and installing dependencies..."
    python3 -m venv .venv
    .venv/bin/pip install --quiet --upgrade pip
    .venv/bin/pip install --quiet -r requirements.txt
fi
exec .venv/bin/python heb_console.py "$@"
