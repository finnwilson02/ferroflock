#!/bin/bash

# Create necessary directory if it doesn't exist
mkdir -p /home/finn/ferroflock

# Check if dji_devices.json exists
if [ -f /home/finn/ferroflock/dji_devices.json ]; then
    # If it exists, make a backup just in case
    cp /home/finn/ferroflock/dji_devices.json /home/finn/ferroflock/dji_devices.json.bak
    
    # Make it writable for our user
    chmod 666 /home/finn/ferroflock/dji_devices.json
    
    echo "Made existing JSON file writable"
else
    # Create an empty JSON file with proper permissions
    echo '{"devices":[]}' > /home/finn/ferroflock/dji_devices.json
    chmod 666 /home/finn/ferroflock/dji_devices.json
    
    echo "Created new JSON file with proper permissions"
fi

# Run the helper_functions executable
echo "Starting helper_functions with proper permissions..."
./helper_functions