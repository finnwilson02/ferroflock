#!/bin/bash
# setup_wifi.bash

echo "Setting up WiFi configurations..."

# Create wpa_supplicant.conf in boot
echo "Creating wpa_supplicant.conf..."
sudo tee /boot/wpa_supplicant.conf > /dev/null << EOF
country=AU
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

# network={
#     ssid-"TELLO-EDF843"
#     priority=4
# }

network={
    ssid="UGVwifi"
    psk="boyimkeen"
    priority=3
}
network={
   ssid="UAV-LAB-2.4G"
   psk="idontknow"
   priority=2
}

network={
   ssid="NETGEAR89"
   psk="greenjade816"
   priority=1
}
EOF

# Copy to correct location
echo "Installing configuration file..."
sudo cp /boot/wpa_supplicant.conf /etc/wpa_supplicant/

# Set correct permissions
echo "Setting permissions..."
sudo chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf

# Restart networking
echo "Restarting network services..."
sudo systemctl restart wpa_supplicant

# Show current network info
echo -e "\nCurrent network configuration:"
echo "--------------------------------"
echo "WiFi Status:"
iwconfig

echo -e "\nIP Addresses:"
hostname -I

echo -e "\nSaved networks:"
sudo wpa_cli list_networks

echo -e "\nWiFi setup complete! After changing networks you can check connection with:"
echo "iwconfig"
echo "hostname -I"