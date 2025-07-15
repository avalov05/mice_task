#!/bin/bash

echo "===== Setting up your Raspberry Pi environment ====="

# Update system packages
sudo apt update && sudo apt upgrade -y

# Enable I2C and SPI interfaces
echo "Enabling I2C and SPI..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0

# Install system-level dependencies
echo "Installing system libraries..."
sudo apt install -y \
    python3-pip \
    python3-venv \
    libatlas-base-dev \
    libjpeg-dev \
    libopenjp2-7 \
    libtiff5 \
    libfreetype6-dev \
    libharfbuzz-dev \
    libfribidi-dev \
    libxcb1-dev \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    libportmidi-dev \
    libswscale-dev \
    libavformat-dev \
    libavcodec-dev \
    libffi-dev \
    libglib2.0-dev \
    libhdf5-dev \
    zlib1g-dev \
    libx11-dev \
    libusb-1.0-0-dev \
    i2c-tools

# Add user to I2C group
sudo usermod -aG i2c $USER

# Create project directory
PROJECT_DIR=~/control_task
mkdir -p $PROJECT_DIR
cd $PROJECT_DIR

# Create and activate virtual environment
echo "Creating Python virtual environment..."
python3 -m venv .venv
source .venv/bin/activate

# Upgrade pip and install Python dependencies inside the venv
echo "Installing Python packages..."
pip install --upgrade pip
pip install \
    adafruit-circuitpython-servokit \
    adafruit-circuitpython-ads1x15 \
    adafruit-circuitpython-mcp4725 \
    gpiozero \
    pygame \
    pillow \
    matplotlib \
    numpy \
    scipy

# Save requirements
pip freeze > requirements.txt

# Done
echo "===== Setup complete! ====="
echo "Your virtual environment is located at: $PROJECT_DIR/.venv"
echo "To activate it later, run:"
echo "  source $PROJECT_DIR/.venv/bin/activate"
echo "Then you can run your script with:"
echo "  python your_script.py"
echo "Remember to reboot to finalize I2C/SPI changes: sudo reboot"
