# 02 - Simulating Physics, Gravity, and Collisions in Gazebo

## Introduction to Physics Simulation in Gazebo

Gazebo is a powerful 3D robotics simulator that accurately and efficiently simulates complex robotic systems in challenging indoor and outdoor environments. A core strength of Gazebo is its robust physics engine, which allows for realistic interactions between rigid bodies, including gravity, collisions, and various joint types.

### Key Physics Concepts in Simulation

*   **Rigid Body Dynamics**: Gazebo treats robot links and environmental objects as rigid bodies, meaning they do not deform. Their motion is governed by forces and torques.
*   **Physics Engines**: Gazebo integrates with several high-performance physics engines like ODE (Open Dynamics Engine), Bullet, DART, and Simbody. These engines solve the equations of motion for all bodies in the simulation.
*   **Gravity**: A fundamental force affecting all objects with mass. Gazebo allows configuring gravity's direction and magnitude.
*   **Collisions**: Occur when two or more simulated objects come into contact. Gazebo's collision detection algorithms determine when contact happens, and the physics engine calculates the resulting forces to prevent interpenetration and simulate realistic bounces or slides.
*   **Friction**: Resistance to motion when objects slide or roll against each other. Both static and dynamic friction coefficients can be defined.

## Configuring Physics in Gazebo

Gazebo's physics properties are typically defined within the world file (`.world`) or model files (`.sdf` - Simulation Description Format).

```mermaid
graph TD
    A[Start: Define Physics Properties] --> B{Global Physics Settings in .world};
    B --> C{Gravity};
    B --> D{Physics Engine Selection (ODE, Bullet)};
    D --> E{Engine Parameters (Max Step Size, Real-time Factor, Update Rate)};
    C --> F{Model Specific Properties in .sdf};
    E --> F;
    F --> G{Inertial Properties (Mass, Inertia)};
    F --> H{Collision Geometry (Box, Sphere, Cylinder)};
    F --> I{Surface Properties (Friction, Bounce)};
    G --> J[End: Realistic Simulation Behavior];
    H --> J;
    I --> J;
```

### World File Configuration (Example)

In a Gazebo world file, you can specify the global physics engine and gravity settings:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.81</gravity> <!-- Earth's gravity in Z-direction -->

    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size> <!-- Simulation time step -->
      <real_time_factor>1.0</real_time_factor> <!-- Run in real-time -->
      <real_time_update_rate>1000</real_time_update_rate> <!-- Physics updates per second -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- ... other world elements ... -->
  </world>
</sdf>
```

**Explanation of parameters**:
*   `<gravity>`: Defines the gravity vector (x y z).
*   `<physics>`: Configures the physics engine. `type` can be `ode`, `bullet`, `dart`, `simbody`.
*   `<max_step_size>`: The maximum time step for the physics engine. Smaller values lead to more accurate but slower simulations.
*   `<real_time_factor>`: Ratio of simulated time to real time. `1.0` means simulation runs at real-time speed.
*   `<real_time_update_rate>`: How many times per second the physics engine updates.
*   `<ode>`: Specific configurations for the ODE physics engine, including solver parameters and constraint settings.

### Model Collision and Inertia Properties (Example: SDF)

For individual models (robot links, objects), collision shapes, inertia, and material properties are crucial for realistic physics.

```xml
<link name="my_box_link">
  <inertial>
    <mass>1.0</mass> <!-- Mass in kg -->
    <inertia>
      <ixx>0.001666667</ixx> <!-- Moment of inertia about x-axis -->
      <iyy>0.001666667</iyy>
      <izz>0.001666667</izz>
    </inertia>
  </inertial>
  <collision name="my_box_collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size> <!-- Size of collision box (10cm cube) -->
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.5</mu> <!-- Coefficient of friction -->
          <mu2>0.5</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.5</restitution_coefficient> <!-- Bounciness -->
        <threshold>0.01</threshold>
      </bounce>
    </surface>
  </collision>
  <visual name="my_box_visual">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
  </visual>
</link>
```

**Explanation**:
*   `<inertial>`: Defines the mass and inertia tensor, critical for how the object responds to forces.
*   `<collision>`: Specifies the collision geometry (e.g., `<box>`, `<cylinder>`, `<sphere>`). This shape is used by the physics engine for collision detection. It's often simplified compared to the visual mesh for performance.
*   `<surface>`: Configures physical surface properties like friction (`<mu>` for primary, `<mu2>` for secondary axis) and bounce (`<restitution_coefficient>`).
*   `<visual>`: Defines the visual appearance of the object. This is what you see in Gazebo's renderer and does not directly affect physics.

## Practical Simulation Example: Falling Boxes

Let's create a simple Gazebo world where several boxes fall onto a plane and interact with each other.

### Setup

1.  **Create a new Gazebo world file**: Save the following content as `falling_boxes.world` in your ROS 2 workspace (e.g., `~/ros2_ws/src/my_gazebo_configs/worlds/`).

    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="falling_boxes_world">
        <gravity>0 0 -9.81</gravity>
        <physics name="default_physics" default="true" type="ode">
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1.0</real_time_factor>
          <real_time_update_rate>1000</real_time_update_rate>
          <ode>
            <solver>
              <type>quick</type>
              <iters>50</iters>
              <sor>1.3</sor>
            </solver>
            <constraints>
              <cfm>0</cfm>
              <erp>0.2</erp>
            </constraints>
          </ode>
        </physics>

        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <!-- Box 1 -->
        <model name="box_1">
          <pose>0 0 5 0 0 0</pose> <!-- x y z roll pitch yaw -->
          <link name="link">
            <inertial>
              <mass>1.0</mass>
              <inertia>
                <ixx>0.001666667</ixx> <ixy>0</ixy> <ixz>0</ixz>
                <iyy>0.001666667</iyy> <iyz>0</iyz>
                <izz>0.001666667</izz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
              <surface>
                <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
                <bounce><restitution_coefficient>0.3</restitution_coefficient></bounce>
              </surface>
            </collision>
            <visual name="visual">
              <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
              <material><script><uri>file://media/materials/scripts/Gazebo.material</uri><name>Gazebo/Red</name></script></material>
            </visual>
          </link>
        </model>

        <!-- Box 2 -->
        <model name="box_2">
          <pose>0.05 0.05 4.5 0 0.1 0</pose>
          <link name="link">
            <inertial>
              <mass>1.0</mass>
              <inertia>
                <ixx>0.001666667</ixx> <ixy>0</ixy> <ixz>0</ixz>
                <iyy>0.001666667</iyy> <iyz>0</iyz>
                <izz>0.001666667</izz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
              <surface>
                <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
                <bounce><restitution_coefficient>0.3</restitution_coefficient></bounce>
              </surface>
            </collision>
            <visual name="visual">
              <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
              <material><script><uri>file://media/materials/scripts/Gazebo.material</uri><name>Gazebo/Blue</name></script></material>
            </visual>
          </link>
        </model>
      </world>
    </sdf>
    ```

2.  **Launch the simulation**:
    ```bash
    gazebo --verbose falling_boxes.world
    ```
    (Ensure you are in the directory containing `falling_boxes.world` or have set up `GAZEBO_MODEL_PATH` correctly for Gazebo to find it.)

### Expected Outcome

You should observe two red and blue boxes falling under gravity onto the ground plane, bouncing slightly, and then coming to rest. Their interaction and final positions will be determined by their mass, inertia, friction, and restitution coefficients.

## Conclusion

Understanding and correctly configuring physics properties is fundamental to creating realistic and accurate simulations in Gazebo. By carefully defining gravity, collision geometries, mass, inertia, and surface characteristics, you can build virtual environments that closely mimic real-world physical interactions, providing a robust platform for robotics research and development. The next section will explore high-fidelity rendering and human-robot interaction in Unity, offering complementary strengths for digital twin visualization.