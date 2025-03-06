#!/bin/bash
# install_torch.bash

echo "Installing PyTorch for Raspberry Pi..."

# Make sure we're in the venv
source ~/drone_venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
sudo apt update
sudo apt install -y python3-pip libjpeg-dev libopenblas-dev libopenmpi-dev libomp-dev

# Install PyTorch
echo "Installing PyTorch..."
pip install --no-cache-dir torch torchvision torchaudio

# Optional but recommended for deep learning
echo "Installing additional ML packages..."
pip install --no-cache-dir \
    scikit-learn \
    tensorboard

# Test installation
echo "Testing PyTorch installation..."
python3 - << EOF
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"Device: {torch.device('cpu')}")

# Quick test tensor operation
x = torch.rand(5, 3)
print("Test tensor:")
print(x)
EOF

echo "PyTorch installation complete!"