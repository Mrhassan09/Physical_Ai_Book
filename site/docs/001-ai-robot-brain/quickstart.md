# Quickstart: The AI-Robot Brain (NVIDIA Isaac™)

This `quickstart.md` provides a high-level overview and essential setup instructions for replicating the NVIDIA Isaac development environment and examples presented in "The AI-Robot Brain (NVIDIA Isaac™)" module. Following these steps will ensure your development environment is ready to explore advanced perception, training, and navigation for AI-driven robotics.

## 1. Prerequisites

Before you begin, ensure you have the following software and hardware components:

*   **NVIDIA GPU**: A modern NVIDIA GPU (e.g., RTX series) is highly recommended for Isaac Sim and Isaac ROS due to their hardware acceleration requirements. NVIDIA Jetson devices are also supported for edge deployments.
*   **Operating System**: Ubuntu 20.04 LTS (Focal Fossa) or Ubuntu 22.04 LTS (Jammy Jellyfish). These are the primary supported operating systems for ROS 2 and NVIDIA Isaac platforms.
*   **ROS 2 Distribution**:
    *   **Humble Hawksbill (LTS)**: Recommended for stability and long-term support. Follow the official installation guide for [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
    *   **Iron Irwini**: For the latest features. Follow the official installation guide for [ROS 2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).
*   **NVIDIA Omniverse Launcher**: Download and install from the [NVIDIA Developer website](https://developer.nvidia.com/omniverse/download). This is required to install and manage Isaac Sim.
*   **NVIDIA Isaac Sim**: Install through the Omniverse Launcher. Refer to the official [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/install_basic_setup.html) for detailed setup.
*   **NVIDIA Isaac ROS**: Follow the installation instructions for [Isaac ROS](https://nvidia-isaac-ros.github.io/getting_started/getting_started.html). This typically involves setting up a Docker environment and pulling pre-built Isaac ROS containers.
*   **Python**: Python 3.8+ (typically comes with Ubuntu 20.04/22.04). Ensure your Python environment is set up for ROS 2.
*   **C++**: C++17 compiler (e.g., g++-9 or newer, typically available on Ubuntu). Required for ROS 2 and Isaac ROS development.
*   **Docker & NVIDIA Container Toolkit**: Essential for running Isaac ROS and other containerized NVIDIA applications. Follow the [Docker installation guide](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA Container Toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
*   **Visual Studio Code (Optional but Recommended)**: For code editing in Python and C++. Install extensions for Python, C++, ROS, and Docker.

## 2. Environment Setup

### 2.1. ROS 2 Workspace Setup

1.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
2.  **Build your workspace (initially empty)**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
    This workspace will be used for your ROS 2 packages and C++/Python code.

### 2.2. NVIDIA Isaac ROS Workspace (Docker)

Isaac ROS typically runs within Docker containers. After installing Docker and NVIDIA Container Toolkit:

1.  **Clone the Isaac ROS container repository**:
    ```bash
    git clone https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_common.git -b humble # or iron
    cd isaac_ros_common
    ```
2.  **Build the base container image**:
    ```bash
    ./scripts/build_base_image.sh
    ```
3.  **Launch the development container**:
    ```bash
    ./scripts/run_dev.sh
    ```
    This will place you inside a Docker container with Isaac ROS pre-installed and configured.

### 2.3. NVIDIA Isaac Sim Project Setup

Isaac Sim is installed via Omniverse Launcher. Once installed:

1.  **Launch Isaac Sim**: Open the Omniverse Launcher, navigate to "Exchange", find "Isaac Sim", and click "Launch".
2.  **Create a new project**: Inside Isaac Sim, you can create new simulation environments or load existing ones. Scripting is primarily done via Python API.

## 3. First Simulation/Code Example: Verification

### 3.1. Isaac ROS "Hello World" (within Docker)

Inside your Isaac ROS development container:

1.  **Run a simple ROS 2 example**:
    ```bash
    ros2 run demo_nodes_cpp talker
    ros2 run demo_nodes_py listener
    ```
    You should see the `talker` publishing messages and the `listener` receiving them.

### 3.2. Isaac Sim "Hello World"

1.  **Launch Isaac Sim**.
2.  **Open the Script Editor**: Go to `Window -> Script Editor`.
3.  **Run a simple Python script**:
    ```python
    import carb
    carb.log_info("Hello from Isaac Sim Python Script!")
    ```
    You should see "Hello from Isaac Sim Python Script!" in the console output.

## Next Steps

With your NVIDIA Isaac environments set up and verified, you are ready to delve into the detailed sections of the module, covering specific topics in Isaac Platform capabilities, Isaac Sim for synthetic data, Isaac ROS for VSLAM and navigation, and Nav2 for humanoid path planning.

