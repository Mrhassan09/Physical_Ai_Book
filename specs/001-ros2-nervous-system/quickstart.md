# Quickstart Guide: Setting Up Your ROS 2 Environment

**Created**: 2025-12-07

This guide provides the essential steps to set up a ROS 2 Jazzy Jalisco environment on Ubuntu 24.04, which is the target platform for this book's examples.

## 1. Install ROS 2 Jazzy Jalisco

First, ensure your system is up-to-date and has the required locales set.

```bash
# Update your system
sudo apt update && sudo apt upgrade

# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Next, add the ROS 2 APT repository to your system.

```bash
# Add the ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Now you can install the recommended ROS 2 Jazzy desktop package.

```bash
# Install ROS 2
sudo apt update
sudo apt install ros-jazzy-desktop

# Install development tools
sudo apt install ros-dev-tools
```

## 2. Set Up Environment

After installation, you need to source the ROS 2 setup file in your terminal. This makes ROS 2 commands available.

```bash
# Source the setup file
source /opt/ros/jazzy/setup.bash
```

It's recommended to add this command to your `~/.bashrc` file to run it automatically in every new terminal session.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 3. Create a Colcon Workspace

A "workspace" is a directory where you can create, modify, and build your own ROS 2 packages.

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (it's empty for now, but this is good practice)
colcon build
```

## 4. Run a "Hello World" Example

To verify your installation, you can run a simple talker/listener example.

Open two separate terminals. In both terminals, navigate to your workspace and source the setup files:

```bash
# Run this in BOTH terminals
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
```

**In the first terminal, run the talker node:**

```bash
ros2 run demo_nodes_cpp talker
```

You should see output like this, indicating it's publishing messages:
```
[INFO] [1670451387.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1670451388.123456789] [talker]: Publishing: 'Hello World: 2'
...
```

**In the second terminal, run the listener node:**

```bash
ros2 run demo_nodes_py listener
```

You should see the messages being received from the talker:
```
[INFO] [1670451387.623456789] [listener]: I heard: [Hello World: 1]
[INFO] [1670451388.623456789] [listener]: I heard: [Hello World: 2]
...
```

If both nodes run and communicate successfully, your ROS 2 environment is ready.
