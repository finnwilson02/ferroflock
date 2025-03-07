# OptiTrack VRPN Data Setup Guide

This guide explains how to set up the OptiTrack system to receive VRPN data for real-time tracking, either on the **lab PC** or your own PC. Follow the instructions to ensure you can capture tracking data and process it via C++ or Python.

---

## Lab Instructions for OptiTrack Setup

1. **Location of the OptiTrack system**:
   - The **OptiTrack cupboard** is located to the left of the orange power board, next to the door to the UAV lab. The code for the lock is **1008**.
   - **Reminder**: Lock the cupboard after use.

2. **Powering up the system**:
   - Turn on the **OptiTrack system** before turning on the PC.
   - At the back wall, switch on **CB3** on the **orange power panel**, and the **hanging powerboard** labeled "OptiTrack."
   
3. **Turn on the PC**:
   - The Optitrack PC can now be turned on - there is no password required

4. **Connect to the correct WiFi**:
   - Network: `UAVLab 2.4G`
   - Password: `idontknow`.
   - The Optitrack PC should be connected via ethernet, but you will need to connect your **UAVs** and **Client PC** to the lab network, as university firewalls won't allow any of this :)

5. **Configure Motive for VRPN Streaming**:
   - Open **Motive** on the PC.
   - Ensure **Streaming** is enabled.
   - Enable **VRPN**, enable **Rigid Bodies**, and enable the bodies you want to broadcast.
   - Use the broadcast address in your code (default is `192.168.1.100`).

6. **Reminder after use**:
   - **Turn off** the PC and the **OptiTrack powerboard**.
   - Lock the OptiTrack cupboard before leaving.

---

## Instructions for Getting VRPN Data on Your Own PC or Lab PC

There are two ways to get VRPN data: via **C++** or **Python**. Follow the instructions for your preferred method.

### 1. Setting up C++ to Receive VRPN Data

#### Prerequisites:
- Install the required packages:
  ```bash
  sudo apt update
  sudo apt install cmake build-essential libboost-all-dev libx11-dev libxext-dev g++ git
  ```
- Install VRPN:
  ``` bash
  git clone https://github.com/vrpn/vrpn.git
  cd vrpn
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

#### Building and Running the C++ Code

1. Clone the project repository:
    ```bash
    git clone https://github.com/finnwilson02/squawkblock.git)
    cd squawkblock
    ```

2. Ensure the tracker address in **vrpn_test.cpp** and **vrpn_client.cpp**, by default **Test@192.168.1.100:3883**, matches the name of the rigid body (**Test**), the IP of the Optitrack server (**192.168.1.100**) and the VRPN port in Motive (**3883**).
    
3. Build the C++ VRPN tracker:
    ```bash
    g++ -o vrpn_test vrpn_test.cpp -lvrpn -lquat
    ```

4. Run the C++ VRPN tracker
    ```bash
    ./vrpn_test
    ```

### 2. Setting up Python to Receive VRPN Data

Python doesn't work very nicely with VRPN, so we will use a **socket** to get the data from C++ to Python.

#### Prerequisites:

- Set up a Python virtual environment
    ```bash
    python3 -m venv vrpn_venv
    source vrpn_venv/bin/activate
    ```

- Install Python dependencies
    ```bash
    pip install [none right now but there will be soon]
    ```

#### Building and Running the C++ Client

1. Build the C++ VRPN Tracker and Broadcaster
    ```bash
    g++ -o vrpn_client vrpn_client.cpp -lvrpn -lquat
    ```

2. Run the C++ VRPN Tracker and Broadcaster
    ```bash
    ./vrpn_client
    ```

#### Running the Python Socket Client
1. Open a new terminal, activate the virtual environment, and run the Python client:
    ```bash
    cd ~/squawkblock/squawkblock/python
    source vrpn_venv/bin/activate
    python vrpn_socket.py
    ```
