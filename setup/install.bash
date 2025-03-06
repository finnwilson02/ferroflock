#!/bin/bash

echo "Starting installation..."

# 1. System updates and dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y build-essential cmake libboost-all-dev libx11-dev libxext-dev g++ git

# 2. Install VRPN
echo "Installing VRPN..."
cd ~
git clone https://github.com/vrpn/vrpn.git
cd vrpn
mkdir build
cd build
cmake ..
make
sudo make install
cd ~

# 3. Python environment (you already have this part)
echo "Setting up Python environment..."
python3 -m venv ~/drone_venv
source ~/drone_venv/bin/activate

# 4. Python packages
echo "Installing Python packages..."
pip install --upgrade pip
pip install -r ~/squawkblock//squawkblock/setup/requirements.txt

# 5. Setup bashrc
echo "Configuring .bashrc..."
if ! grep -q "source ~/drone_venv/bin/activate" ~/.bashrc; then
    echo "source ~/drone_venv/bin/activate" >> ~/.bashrc
fi

echo "Installation complete! Please restart your terminal."