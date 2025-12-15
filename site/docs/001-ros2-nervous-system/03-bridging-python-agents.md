---
sidebar_position: 3
---

# 3. Bridging Python Agents to ROS 2

One of the greatest strengths of ROS 2 is its support for multiple programming languages. While C++ is often used for performance-critical components, Python is the language of choice for many in the AI and robotics community due to its ease of use and extensive library ecosystem.

This section focuses on `rclpy`, the official Python client library for ROS 2. It allows you to interface with the entire ROS 2 ecosystem—creating nodes, publishing and subscribing to topics, and calling services—all from a simple Python script.

## What is `rclpy`?

`rclpy` (ROS Client Library for Python) is a library that implements the ROS 2 client API in Python. It is built on top of the ROS 2 C libraries (`rcl`) and the underlying DDS (Data Distribution Service) communication layer. This layered architecture allows Python developers to access all the core features of ROS 2 without needing to write C++ code.

Using `rclpy`, you can write a ROS 2 node entirely in Python. This is particularly powerful for tasks involving:
-   **Artificial Intelligence**: Integrating popular libraries like TensorFlow, PyTorch, or Scikit-learn for perception and decision-making.
-   **Rapid Prototyping**: Quickly scripting new robot behaviors and testing ideas.
-   **System Integration**: Writing scripts to manage, monitor, and debug a complex ROS 2 system.

## Creating Your First Python Node

Let's walk through the fundamental steps to create a ROS 2 node in Python. Every `rclpy` application follows a similar pattern.

### 1. Initialization
First, you must initialize the `rclpy` library. This is done once per process.

```python
import rclpy

def main(args=None):
    rclpy.init(args=args)
    # ... your node logic here ...
    rclpy.shutdown()
```

### 2. Creating a Node
Next, you create an instance of a `Node`. It's common practice to create a class that inherits from `rclpy.node.Node`.

```python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_python_node')
```

### 3. Implementing Logic (Publishers, Subscribers, etc.)
Inside your node's `__init__` method, you can create publishers, subscribers, and other ROS 2 entities.

For example, to create a publisher that sends string messages to a `chatter` topic:
```python
from std_msgs.msg import String

# Inside the MyNode class __init__ method
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

### 4. Spinning the Node
A node needs to be "spun" to keep it alive and allow it to process callbacks (like receiving messages). The `rclpy.spin()` function blocks and processes events for a given node.

```python
def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()
```

In the following examples, we will see how to put these pieces together to create simple publisher and subscriber nodes.

## Publisher Example

Here is the full code for a simple publisher node. This node will publish a "Hello World" message with a counter to the `chatter` topic once per second.

Save this code as `examples/simple_publisher.py` in your package.

```python title="examples/simple_publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    A simple ROS 2 publisher node that publishes a "Hello World" message
    every second to the 'chatter' topic.
    """
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher node started. Publishing to "chatter" topic...')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        simple_publisher = SimplePublisher()
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if 'simple_publisher' in locals() and rclpy.ok():
            simple_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```


### How to Run the Publisher

To run this node, navigate to the root of your Docusaurus project and use the `ros2 run` command, specifying your package name and the name of the executable (which is the name of your Python file without the `.py` extension if you set it up with a `setup.py` file, or you can run it directly with `python3`). For simplicity, you can run it directly from the `site` folder:

```bash
# From the 'site' directory
ros2 run your_package_name simple_publisher
```
*Note: You would need a `package.xml` and `setup.py` to run this with `ros2 run`. For now, you can also run it as a standard python script after sourcing your ROS 2 environment: `python3 docs/001-ros2-nervous-system/examples/simple_publisher.py`*

## Subscriber Example

Now, let's create a subscriber node to listen to the `chatter` topic and print the messages it receives.

Save this code as `examples/simple_subscriber.py`.

```mdx-code-block
import CodeBlock from '@theme/CodeBlock';
import SimpleSubscriber from '!!raw-loader!./examples/simple_subscriber.py';

<CodeBlock language="python" title="examples/simple_subscriber.py">
  {SimpleSubscriber}
</CodeBlock>
```

### How to Run the Subscriber

Open a new terminal, source your ROS 2 environment, and run the subscriber node.

```bash
# From the 'site' directory
ros2 run your_package_name simple_subscriber
```

If the publisher is running, you will see the "I heard: ..." messages printed to the console. This demonstrates a complete, one-way communication link between two Python nodes using ROS 2 topics.

### Python Nodes in the ROS Graph

The following diagram shows how the two nodes we just created fit into the ROS graph.

```mermaid
graph LR
    subgraph "ROS 2 Graph"
        direction LR
        Publisher[simple_publisher (Python Node)]
        Subscriber[simple_subscriber (Python Node)]
        Topic((chatter Topic <br> [std_msgs/msg/String]))
    end

    Publisher -- Publishes --> Topic
    Topic -- "Hello World: ..." --> Subscriber

    style Publisher fill:#f9f,stroke:#333,stroke-width:2px
    style Subscriber fill:#f9f,stroke:#333,stroke-width:2px
    style Topic fill:#ccf,stroke:#333,stroke-width:2px
```

This ability to quickly script ROS 2 nodes in Python is one of the most compelling features for rapid development and for integrating with the vast ecosystem of Python-based AI and robotics libraries (Open Robotics, n.d.).

---
### References

Open Robotics. (n.d.). *Why are there ROS 2 client libraries?*. ROS 2 Documentation. Retrieved December 7, 2025, from https://docs.ros.org/en/rolling/Concepts/Client-Libraries/Why-ROS-2-Client-Libraries.html
