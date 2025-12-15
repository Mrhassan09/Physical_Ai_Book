# Quickstart: The Digital Twin (Gazebo & Unity)

This `quickstart.md` provides a high-level overview and essential setup instructions for replicating the simulation environments and examples presented in "The Digital Twin (Gazebo & Unity)" module. Following these steps will ensure your development environment is ready to explore digital twin concepts, Gazebo simulations, and Unity rendering.

## 1. Prerequisites

Before you begin, ensure you have the following software installed:

*   **Operating System**: Ubuntu 20.04 LTS (Focal Fossa) or newer for ROS 2 and Gazebo. Windows 10/11 for Unity. (Dual-boot or VM for Ubuntu is recommended on Windows machines.)
*   **ROS 2**: Foxy Fitzroy or Galactic Geochelone distribution (recommended). Follow the official installation guide for [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).
*   **Gazebo**: Gazebo Garden (part of Ignition Gazebo) is recommended for modern ROS 2 integration. Install via the official [Ignition Gazebo documentation](https://ignitionrobotics.org/docs/garden/install_ubuntu).
*   **Unity Hub**: Download and install Unity Hub from the [Unity website](https://unity3d.com/get-unity/download).
*   **Unity Editor**: Install Unity Editor version 2021.3 LTS or newer through Unity Hub. Ensure the "Linux Build Support (Mono)" and "Windows Build Support (Mono)" modules are installed if you plan to build applications for those platforms.
*   **Python**: Python 3.8+ (typically pre-installed on Ubuntu 20.04+). Ensure it's correctly linked with your ROS 2 environment.
*   **C#**: Integrated with Unity. No separate installation needed.
*   **Visual Studio Code (Optional but Recommended)**: For code editing in Python and C#. Install extensions for Python, C#, and ROS.

## 2. Environment Setup

### 2.1. ROS 2 and Gazebo Integration (Ubuntu)

1.  **Source your ROS 2 environment**:
    ```bash
    source /opt/ros/foxy/setup.bash # or galactic/setup.bash
    ```
2.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
3.  **Build your workspace (initially empty)**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
    This workspace will be used for ROS 2 packages and Python scripts interacting with Gazebo.

### 2.2. Unity Project Setup (Windows/Ubuntu)

1.  **Create a new Unity Project**:
    *   Open Unity Hub.
    *   Click "New Project".
    *   Select a "3D (URP)" or "3D" template.
    *   Choose a project name (e.g., `DigitalTwinUnity`) and location.
    *   Click "Create project".
2.  **Install ROS-Unity Integration (Optional but Recommended)**:
    *   If planning extensive ROS 2 and Unity integration, consider the [ROS-Unity Integration](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) package. Follow its installation instructions.

## 3. First Simulation: Verification

### 3.1. Gazebo "Hello World"

1.  **Launch an empty Gazebo world**:
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py empty_world.launch.xml
    ```
    You should see an empty Gazebo simulation environment.
2.  **Spawn a simple model (e.g., a cube)**:
    ```bash
    ros2 run gazebo_ros spawn_entity.py -entity my_cube -file `ros2 pkg prefix gazebo_models`/share/gazebo_models/models/cube/model.sdf
    ```
    A cube should appear in your Gazebo world.

### 3.2. Unity "Hello World"

1.  **Create a simple 3D object**:
    *   In your Unity project, right-click in the Hierarchy window.
    *   Select "3D Object" -> "Cube".
    *   Position it at (0,0,0).
2.  **Run the scene**:
    *   Click the "Play" button in the Unity Editor.
    *   The scene should run, and you should see your cube.

## Next Steps

With your environments set up and verified, you are ready to delve into the detailed sections of the module, covering specific topics in Gazebo simulation, Unity rendering, and sensor modeling.

