# Research Findings: The AI-Robot Brain (NVIDIA Isaac™)

## Research Topic: Specific Python and C++ versions compatible with latest NVIDIA Isaac platforms and ROS 2 distributions

### Decision:
*   **Python Version**: Python 3.8+ (for Isaac Sim, Isaac ROS, ROS 2)
*   **C++ Version**: C++17 (for Isaac ROS, Nav2, ROS 2)
*   **ROS 2 Distribution**: Humble Hawksbill (LTS) or Iron Irwini (latest). Humble is preferred for stability and long-term support.
*   **NVIDIA Isaac Sim**: Uses Omniverse Kit Python API, generally compatible with Python 3.7-3.9 (check specific Isaac Sim version documentation for precise compatibility).
*   **NVIDIA Isaac ROS**: Built on ROS 2, leveraging C++ and Python. Compatible with ROS 2 Humble/Iron.

### Rationale:
**Python 3.8+**: Modern NVIDIA Isaac platforms, including Isaac Sim and Isaac ROS, are heavily integrated with Python. ROS 2 also predominantly uses Python 3 for its client libraries (rclpy). Python 3.8 and newer versions offer features and performance improvements beneficial for AI and robotics development. Isaac Sim's scripting API is Python-based.

**C++17**: ROS 2 core components and many high-performance packages, especially in Isaac ROS and Nav2, are written in C++. C++17 is the standard version supported across recent ROS 2 distributions (Humble, Iron) and provides modern language features for efficient and maintainable code.

**ROS 2 Humble Hawksbill**: As an LTS (Long-Term Support) release, Humble provides extended support and stability, which is crucial for a textbook aiming for reproducibility and longevity of examples. It is fully compatible with Isaac ROS. Iron Irwini is newer but with shorter support.

**NVIDIA Isaac Sim and Isaac ROS Integration**: Both Isaac Sim and Isaac ROS are designed to work seamlessly with the chosen Python and C++/ROS 2 versions, ensuring that the examples provided in the textbook are functional and up-to-date with the NVIDIA ecosystem.

### Alternatives Considered:
1.  **Older Python/C++ versions**: Would limit access to modern libraries, language features, and potentially introduce compatibility issues with newer Isaac and ROS 2 releases.
2.  **Different ROS 2 Distributions (e.g., Foxy Fitzroy)**: While Foxy is an LTS, Humble and Iron offer more recent features and better compatibility with the latest NVIDIA Isaac offerings. Choosing a newer LTS (Humble) provides a good balance of features and stability.
3.  **Non-standard language versions**: Could lead to environment setup complexities and reproducibility issues for the target audience. The chosen versions are widely adopted and supported in the robotics community and by NVIDIA.

---
