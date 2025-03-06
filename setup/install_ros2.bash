# First, set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists and install ROS2 Iron
sudo apt update
sudo apt upgrade

# Install ROS2 Iron base + development tools
sudo apt install ros-iron-ros-base \
                 ros-dev-tools \
                 python3-colcon-common-extensions \
                 ros-iron-image-transport* \
                 ros-iron-camera-info-manager \
                 ros-iron-image-pipeline

# Add ROS2 setup to your .bashrc
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc