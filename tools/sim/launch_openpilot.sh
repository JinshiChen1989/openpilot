#!/usr/bin/env bash

# Set up Python path for module imports
SCRIPT_DIR=$(dirname "$0")
OPENPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

# Set up Python path (needed for compiled modules)
export PYTHONPATH="$OPENPILOT_ROOT/openpilot:$OPENPILOT_ROOT/msgq_repo:$OPENPILOT_ROOT/opendbc_repo:$OPENPILOT_ROOT:$PYTHONPATH"

# Add MetaDrive to Python path if it exists (for plant model simulation)
if [[ -d "/home/vcar/Winsurf/metadrive" ]]; then
    export PYTHONPATH="/home/vcar/Winsurf/metadrive:$PYTHONPATH"
    echo "MetaDrive plant model available"
fi

# Use virtual environment if it exists
if [[ -f "$OPENPILOT_ROOT/.venv/bin/activate" ]]; then
    source "$OPENPILOT_ROOT/.venv/bin/activate"
    echo "Using virtual environment with PYTHONPATH"
else
    echo "Using system Python with PYTHONPATH"
fi

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"

OPENPILOT_DIR=$OPENPILOT_ROOT

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
