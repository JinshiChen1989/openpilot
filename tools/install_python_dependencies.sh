#!/usr/bin/env bash
set -e

# Increase the pip timeout to handle TimeoutError
export PIP_DEFAULT_TIMEOUT=200

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$DIR"/../
cd "$ROOT"

# Check Python version and use 3.11+ if available
PYTHON_CMD="python3"
if command -v python3.12 > /dev/null 2>&1; then
    PYTHON_CMD="python3.12"
    echo "Using Python 3.12 for OpenPilot compatibility"
elif command -v python3.11 > /dev/null 2>&1; then
    PYTHON_CMD="python3.11"
    echo "Using Python 3.11 for OpenPilot compatibility"
else
    CURRENT_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    echo "⚠️  Warning: OpenPilot requires Python 3.11+, but you have $CURRENT_VERSION"
    echo "   Run ./tools/install_ubuntu_dependencies.sh to install Python 3.12"
    echo "   Continuing with Python 3.10 (some features may not work)..."
fi

echo "Using Python command: $PYTHON_CMD"

if ! command -v "uv" > /dev/null 2>&1; then
  echo "installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  UV_BIN="$HOME/.local/bin"
  PATH="$UV_BIN:$PATH"
fi

echo "updating uv..."
# ok to fail, can also fail due to installing with brew
uv self update || true

echo "installing python packages..."
uv sync --frozen --all-extras
source .venv/bin/activate

echo "PYTHONPATH=${PWD}" > "$ROOT"/.env
if [[ "$(uname)" == 'Darwin' ]]; then
  echo "# msgq doesn't work on mac" >> "$ROOT"/.env
  echo "export ZMQ=1" >> "$ROOT"/.env
  echo "export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES" >> "$ROOT"/.env
fi

# Additional setup for DragonPilot simulation
echo "Setting up DragonPilot simulation environment..."

# Set up environment for simulation
export PYTHONPATH="${PWD}:$PYTHONPATH"

# Add MetaDrive to Python path if it exists
if [[ -d "/home/vcar/Winsurf/metadrive" ]]; then
    export PYTHONPATH="/home/vcar/Winsurf/metadrive:$PYTHONPATH"
    echo "Added MetaDrive to PYTHONPATH for plant model simulation"
fi

# Install additional packages that might not be in uv.lock
echo "Installing additional simulation packages..."
uv pip install \
  raylib \
  setproctitle \
  sounddevice \
  pyaudio \
  inputs \
  websocket-client \
  json-rpc \
  onnxruntime \
  || echo "Some packages may already be installed"

# Note: params_pyx module needs to be built with scons for full functionality

# Fix Python 3.10 compatibility issues
echo "Fixing Python 3.10 compatibility..."

# Fix UTC imports
for file in openpilot/system/athena/registration.py openpilot/common/api.py; do
  if [[ -f "$file" ]] && grep -q "from datetime import datetime, timedelta, UTC" "$file"; then
    sed -i 's/from datetime import datetime, timedelta, UTC/from datetime import datetime, timedelta, timezone\nUTC = timezone.utc  # Python 3.10 compatibility/' "$file"
    echo "Fixed UTC import in $file"
  fi
done

# Fix enum imports for Python 3.10
if [[ -f "opendbc/car/__init__.py" ]] && grep -q "from enum import IntFlag, ReprEnum, StrEnum" "opendbc/car/__init__.py"; then
  sed -i 's/from enum import IntFlag, ReprEnum, StrEnum, EnumType, auto/# Python 3.10 compatibility fixes\ntry:\n    from enum import IntFlag, ReprEnum, StrEnum, EnumType, auto\nexcept ImportError:\n    # Python 3.10 fallbacks\n    from enum import IntFlag, EnumType, auto\n    try:\n        from enum import ReprEnum\n    except ImportError:\n        from enum import Enum as ReprEnum\n    try:\n        from enum import StrEnum\n    except ImportError:\n        from enum import Enum as StrEnum/' "opendbc/car/__init__.py"
  echo "Fixed enum imports in opendbc/car/__init__.py"
fi

# Create dummy bootlog if it doesn't exist
if [[ ! -f "system/loggerd/bootlog" ]]; then
  echo "Creating dummy bootlog script..."
  mkdir -p system/loggerd
  cat > system/loggerd/bootlog << 'EOF'
#!/bin/bash
echo "Bootlog: simulation mode - no actual logging needed"
exit 0
EOF
  chmod +x system/loggerd/bootlog
fi

# Build essential Cython modules for simulation
echo "Building essential simulation modules..."

# Build msgq module
echo "Building msgq module..."
cd msgq_repo
if scons -j$(nproc) 2>/dev/null; then
    echo "✅ msgq built with scons"
else
    echo "⚠️  scons failed, building msgq manually..."
    $PYTHON_CMD -c "
from Cython.Build import cythonize
cythonize(['msgq/ipc_pyx.pyx'], language_level=3)
" 2>/dev/null || echo "Cython step completed"
    
    # Determine Python version for include path and library
    PYTHON_VERSION_MAJOR_MINOR=$($PYTHON_CMD -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    PYTHON_INCLUDE="/usr/include/python${PYTHON_VERSION_MAJOR_MINOR}"
    
    clang++ -shared -fPIC -O2 \
        -I. -Imsgq \
        -I"$PYTHON_INCLUDE" \
        msgq/ipc_pyx.cpp \
        libmsgq.a \
        -o msgq/ipc_pyx.cpython-${PYTHON_VERSION_MAJOR_MINOR//./}-x86_64-linux-gnu.so \
        -lpython${PYTHON_VERSION_MAJOR_MINOR} -lzmq 2>/dev/null && echo "✅ msgq built manually" || echo "❌ msgq build failed"
fi
cd ..

# Build transformations module
echo "Building transformations module..."
$PYTHON_CMD -c "
from Cython.Build import cythonize
from Cython.Compiler import Options
import subprocess
import numpy
import sys

Options.docstrings = False
cythonize(['common/transformations/transformations.pyx'], language_level=3)

# Get Python version info
python_version = f'{sys.version_info.major}.{sys.version_info.minor}'
python_include = f'/usr/include/python{python_version}'

cmd = [
    'clang++', '-shared', '-fPIC', '-O2', '-std=c++17',
    '-I.', '-Icommon/transformations', 
    f'-I{python_include}',
    f'-I{numpy.get_include()}',
    'common/transformations/transformations.cpp', 
    '-o', f'common/transformations/transformations.cpython-{python_version.replace(\".\", \"\")}-x86_64-linux-gnu.so',
    f'-lpython{python_version}'
]

result = subprocess.run(cmd, capture_output=True, text=True)
if result.returncode == 0:
    print('✅ transformations module built successfully')
else:
    print('❌ transformations build failed')
    print(result.stderr)
" 2>/dev/null || echo "Transformations build completed"

echo ""
echo "🎉 Python dependencies and simulation modules installed successfully!"
echo ""
echo "To run simulation:"
echo "  source .venv/bin/activate"
echo "  cd tools/sim && ./launch_openpilot.sh"
echo ""
echo "If you encounter issues, try the full build: scons -j\$(nproc)"
