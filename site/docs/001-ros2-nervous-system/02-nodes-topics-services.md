---
sidebar_position: 2
---

# 2. ROS 2 Core Concepts

Now that we understand the "why" of ROS 2, let's explore the "what." The ROS 2 ecosystem is built upon a few core concepts that work together to create a powerful and flexible communication system. These are the fundamental building blocks of any ROS 2 application.

## The ROS 2 Graph

Imagine all the software running on a robot as a network of interconnected processes. This is what we call the **ROS Graph**. It's a conceptual representation of all the ROS 2 components and the communication pathways between them. The main elements of this graph are Nodes, Topics, and Services.

## Nodes: The Building Blocks

A **Node** is the smallest computational unit in ROS 2. Think of a node as a single, focused program responsible for one specific task. For example, you might have:

-   A `camera_driver` node that captures images from a camera.
-   An `object_detection` node that processes images to find objects.
-   A `motor_controller` node that sends commands to the wheels.

Each node is an independent executable. You can run them on the same machine or distribute them across multiple computers on a network. This modularity is a key strength of ROS 2, as it allows for fault isolation and easy replacement or upgrading of individual components.

## Topics: The Public Announcement System

**Topics** are named buses that facilitate anonymous, one-way communication. They follow a **publish-subscribe** model.

-   **Publishers**: A node can "publish" messages (data) to a topic. It doesn't know or care who is listening.
-   **Subscribers**: A node can "subscribe" to a topic to receive messages published to it. It doesn't know or care who sent the message.

This decoupling is powerful. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to it, creating a many-to-many communication highway.

**Example**: A `/camera/image` topic might have a `camera_driver` node publishing images to it. Both an `object_detection` node and a `video_recorder` node could subscribe to this same topic to receive the images simultaneously.

Every message on a topic must conform to a specific **Message Type**, which defines the data structure (e.g., `sensor_msgs/msg/Image`).

## Services: The Question-and-Answer System

While topics are great for continuous data streams, they aren't suitable for request/reply interactions. For this, ROS 2 provides **Services**.

A service is a two-way communication mechanism where one node (the **server**) offers a function, and another node (the **client**) can call that function and receive a response. Unlike topics, services are synchronous: the client sends a request and waits until the server provides a response.

**Example**: You might have a `/spawn_robot` service. A simulation control node (the client) would send a request to a Gazebo simulation node (the server) with the robot's desired location. The server would then create the robot in the simulation and send back a response indicating whether the operation was successful.

Like topics, services have a defined **Service Type** that specifies the structure of both the request and the response.

### Visualizing Communication

#### Publish-Subscribe Model (Topics)

```mermaid
graph LR
    subgraph " "
        direction LR
        Publisher1[Node A <br> (Publisher)]
        Publisher2[Node B <br> (Publisher)]
        Subscriber1[Node C <br> (Subscriber)]
        Subscriber2[Node D <br> (Subscriber)]
        Topic((Topic: /sensor_data))
    end

    Publisher1 -- Publishes --> Topic
    Publisher2 -- Publishes --> Topic
    Topic -- Message --> Subscriber1
    Topic -- Message --> Subscriber2

    style Topic fill:#ccf,stroke:#333,stroke-width:2px
```

#### Request-Reply Model (Services)

```mermaid
graph LR
    subgraph " "
        direction LR
        Client[Node A <br> (Client)]
        Server[Node B <br> (Server)]
        Service((Service: /compute_task))
    end

    Client -- Request --> Service
    Service -- Request --> Server
    Server -- Response --> Service
    Service -- Response --> Client

    style Service fill:#cfc,stroke:#333,stroke-width:2px
```

All of these concepts are managed by the underlying ROS 2 framework and can be explored and debugged using a rich set of command-line tools (Open Robotics, n.d.).

---
### References

Open Robotics. (n.d.). *Concepts*. ROS 2 Documentation. Retrieved December 7, 2025, from https://docs.ros.org/en/jazzy/Concepts.html


