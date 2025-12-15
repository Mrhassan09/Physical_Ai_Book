# Research: ROS 2 Version Selection

**Created**: 2025-12-07

## Decision: Standardize on ROS 2 Jazzy Jalisco

The project will use **ROS 2 Jazzy Jalisco** as the official version for all examples, instructions, and concepts.

## Rationale

The choice of a ROS 2 distribution is critical for a textbook, as it directly impacts the stability, relevance, and longevity of the content. After researching the available ROS 2 distributions, Jazzy Jalisco was selected for the following reasons:

1.  **Long-Term Support (LTS)**: Jazzy is an LTS release, offering 5 years of support until May 2029. This is the most important factor for a publication, as it ensures the material remains accurate and usable for students and professionals for a significant period.

2.  **Modern and Stable**: As of December 2025, Jazzy has been available for over 18 months, allowing it to mature and stabilize. Most of the initial bugs have been resolved, and a robust ecosystem of community packages and support is available.

3.  **Future-Proofing**: By adopting the latest LTS, the book positions its readers at the forefront of current ROS 2 development, ensuring the skills they learn are relevant for new and ongoing projects.

## Alternatives Considered

| Option | Rationale for Rejection |
|---|---|
| **ROS 2 Humble Hawksbill** | While also an LTS release (supported until 2027), it is two years older than Jazzy. Adopting Humble would mean the book's content would become outdated sooner. |
| **ROS 2 Rolling Ridley** | This is a development distribution and not a stable release. It is unsuitable for a textbook that requires a fixed, reproducible environment. |
| **Non-LTS Releases (e.g., Iron Irwini)** | These releases have a short support lifespan (1.5 years) and are not appropriate for foundational educational material. Iron's support ended in late 2024. |
