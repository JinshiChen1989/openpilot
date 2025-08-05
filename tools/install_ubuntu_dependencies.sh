#!/usr/bin/env bash
set -e

SUDO=""

# Use sudo if not root
if [[ ! $(id -u) -eq 0 ]]; then
  if [[ -z $(which sudo) ]]; then
    echo "Please install sudo or run as root"
    exit 1
  fi
  SUDO="sudo"
fi

# Check if stdin is open
if [ -t 0 ]; then
  INTERACTIVE=1
fi

# Install common packages
function install_ubuntu_common_requirements() {
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends \
    ca-certificates \
    clang \
    build-essential \
    gcc-arm-none-eabi \
    liblzma-dev \
    capnproto \
    libcapnp-dev \
    curl \
    libcurl4-openssl-dev \
    git \
    git-lfs \
    ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libavfilter-dev \
    libbz2-dev \
    libeigen3-dev \
    libffi-dev \
    libglew-dev \
    libgles2-mesa-dev \
    libglfw3-dev \
    libglib2.0-0 \
    libjpeg-dev \
    libqt5charts5-dev \
    libncurses5-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libzmq3-dev \
    libzstd-dev \
    libsqlite3-dev \
    libsystemd-dev \
    locales \
    opencl-headers \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    portaudio19-dev \
    qttools5-dev-tools \
    libqt5svg5-dev \
    libqt5serialbus5-dev  \
    libqt5x11extras5-dev \
    libqt5opengl5-dev \
    xvfb \
    pkg-config \
    python3-pip \
    libasound2-dev \
    libportaudio2

  # Check Python version requirement
  PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
  echo "Current Python version: $PYTHON_VERSION"
  
  if [[ "$PYTHON_VERSION" < "3.11" ]]; then
    echo "⚠️  OpenPilot/DragonPilot requires Python 3.11+ but you have $PYTHON_VERSION"
    echo "Installing Python 3.12..."
    
    # Add deadsnakes PPA for newer Python versions
    $SUDO apt-get install -y software-properties-common
    $SUDO add-apt-repository -y ppa:deadsnakes/ppa
    $SUDO apt-get update
    
    # Install Python 3.12
    $SUDO apt-get install -y \
      python3.12 \
      python3.12-dev \
      python3.12-venv
    
    # Install pip for Python 3.12
    curl -sS https://bootstrap.pypa.io/get-pip.py | $SUDO python3.12
    
    echo "✅ Python 3.12 installed. You should use 'python3.12' instead of 'python3'"
    echo ""
    echo "To set Python 3.12 as default (optional):"
    echo "  sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1"
    echo "  sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 2"
    echo "  sudo update-alternatives --config python3"
  fi

  # Additional packages for simulation and building Cython extensions
  echo "Installing additional packages for simulation..."
  $SUDO apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-setuptools \
    python3-wheel \
    cython3 || echo "Some Python packages may need to be installed via pip"
}

# Install Ubuntu 24.04 LTS packages
function install_ubuntu_lts_latest_requirements() {
  install_ubuntu_common_requirements

  $SUDO apt-get install -y --no-install-recommends \
    g++-12 \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    python3-dev \
    python3-venv
}

# Detect OS using /etc/os-release file
if [ -f "/etc/os-release" ]; then
  source /etc/os-release
  case "$VERSION_CODENAME" in
    "jammy" | "kinetic" | "noble")
      install_ubuntu_lts_latest_requirements
      ;;
    *)
      echo "$ID $VERSION_ID is unsupported. This setup script is written for Ubuntu 24.04."
      read -p "Would you like to attempt installation anyway? " -n 1 -r
      echo ""
      if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
      fi
      install_ubuntu_lts_latest_requirements
  esac

  if [[ -d "/etc/udev/rules.d/" ]]; then
    # Setup jungle udev rules
    $SUDO tee /etc/udev/rules.d/12-panda_jungle.rules > /dev/null <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="3801", ATTRS{idProduct}=="ddcf", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="3801", ATTRS{idProduct}=="ddef", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddcf", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddef", MODE="0666"

EOF

    # Setup panda udev rules
    $SUDO tee /etc/udev/rules.d/11-panda.rules > /dev/null <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="3801", ATTRS{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="3801", ATTRS{idProduct}=="ddee", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddee", MODE="0666"
EOF

    $SUDO udevadm control --reload-rules && $SUDO udevadm trigger || true
  fi

  # Post-installation setup for DragonPilot/OpenPilot
  echo "Setting up DragonPilot environment..."
  
  # Set up Python path for current user
  DRAGONPILOT_ROOT="$(realpath "$(dirname "$0")/..")"
  
  # Add to bashrc if not already present
  if ! grep -q "PYTHONPATH.*dragonpilot" ~/.bashrc 2>/dev/null; then
    echo "" >> ~/.bashrc
    echo "# DragonPilot environment (added by install_ubuntu_dependencies.sh)" >> ~/.bashrc
    echo "export PYTHONPATH=\"$DRAGONPILOT_ROOT:\$PYTHONPATH\"" >> ~/.bashrc
    echo "export PATH=\"\$HOME/.local/bin:\$PATH\"" >> ~/.bashrc
    
    # Add MetaDrive to PYTHONPATH if it exists
    if [[ -d "/home/vcar/Winsurf/metadrive" ]]; then
      echo "export PYTHONPATH=\"/home/vcar/Winsurf/metadrive:\$PYTHONPATH\"  # MetaDrive plant model" >> ~/.bashrc
      echo "MetaDrive path added to environment"
    fi
    
    echo "Environment variables added to ~/.bashrc"
  fi
  
  # Create /dev/shm/params directory with proper permissions
  if [[ ! -d "/dev/shm/params" ]]; then
    $SUDO mkdir -p /dev/shm/params
    $SUDO chmod 777 /dev/shm/params
    echo "Created /dev/shm/params directory for parameter storage"
  fi
  
  echo ""
  echo "✅ Ubuntu dependencies installed successfully!"
  echo ""
  echo "Next steps:"
  echo "1. Install Python dependencies: ./tools/install_python_dependencies.sh"
  echo "2. Build the project: scons -j\$(nproc)"
  echo "3. Or run the complete setup: ./INSTALL_DEPENDENCIES.sh"
  echo ""
  echo "For simulation:"
  echo "  cd tools/sim && ./launch_openpilot.sh"
  echo ""
  echo "⚠️  Start a new terminal or run 'source ~/.bashrc' to load environment variables"

else
  echo "No /etc/os-release in the system. Make sure you're running on Ubuntu, or similar."
  exit 1
fi
