<!--
- Version change: 0.1.0 -> 1.0.0
- List of modified principles:
  - PRINCIPLE_1_NAME -> Accuracy
  - PRINCIPLE_2_NAME -> Clarity
  - PRINCIPLE_3_NAME -> Reproducibility
  - NEW: Rigor
- Added sections:
  - Principle 4: Rigor
- Removed sections:
  - Principle 5
- Templates requiring updates:
  - .specify/templates/plan-template.md (✅ updated)
  - .specify/templates/spec-template.md (✅ updated)
  - .specify/templates/tasks-template.md (✅ updated)
- Follow-up TODOs: None
-->

# Book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### Principle 1: Accuracy

**Rule:** All technical claims, code examples, and instructional steps MUST be verified against primary sources, including official documentation (e.g., ROS 2, Gazebo, NVIDIA Isaac, Unity) and peer-reviewed publications.

**Rationale:** The credibility of this textbook rests on the correctness of its information. Readers, including students and professionals, rely on this material for their learning and work. Any inaccuracy can lead to significant errors in their projects and a loss of trust in this resource.

### Principle 2: Clarity

**Rule:** Content MUST be written for a technical audience, specifically students and professionals in robotics and AI, targeting a Flesch-Kincaid grade level of 10-12. All concepts, examples, and instructions MUST be explained in a clear, direct, and unambiguous manner.

**Rationale:** The primary goal is education. Complex topics must be made accessible without sacrificing technical depth. This ensures the material is understandable to its target audience, facilitating effective learning.

### Principle 3: Reproducibility

**Rule:** All code, simulations, and setup instructions provided in the book MUST be fully reproducible. This includes providing versioned dependencies, environment configurations, and step-by-step guides that have been tested and confirmed to work.

**Rationale:** To bridge the gap between theory and practice, readers must be able to replicate the results and experiments shown in the book. This hands-on experience is critical for genuine understanding and skill development in physical AI.

### Principle 4: Rigor

**Rule:** The material MUST be grounded in academic and professional rigor. At least 50% of the sources used MUST be peer-reviewed articles, official technical documentation, or formal whitepapers. All sources MUST be cited using APA style.

**Rationale:** Rigorous sourcing ensures the content is reliable, up-to-date, and reflects the state-of-the-art in the field. It provides a strong foundation for the book's claims and directs readers to further high-quality information.

## Key Standards

-   **Traceability:** All factual claims must be traceable to official documentation or peer-reviewed articles.
-   **Citation Format:** All sources MUST be cited using APA style.
-   **Source Quality:** A minimum of 50% of sources must be from peer-reviewed articles, official documentation, and technical whitepapers.
-   **Plagiarism:** There is a zero-tolerance policy for plagiarism. All content must pass a plagiarism check before publication.
-   **Writing Clarity:** Content must target a Flesch-Kincaid grade level of 10-12 to ensure it is accessible to the intended technical audience.
-   **Publishing Format:** All content MUST be authored in Markdown and published using Docusaurus to ensure consistent formatting and citation embedding.

## Constraints

-   **Length:** The book's total word count must be between 50,000 and 70,000 words.
-   **Source Count:** A minimum of 30 unique, high-quality sources must be cited.
-   **Output Format:** The primary deliverable is a Docusaurus-generated static website, with a secondary, downloadable PDF version.

## Success Criteria

-   **Verification:** All technical claims are successfully verified against primary sources.
-   **Originality:** Zero plagiarism is detected in the final manuscript.
-   **Reproducibility:** All code examples and simulation steps are confirmed to be reproducible by a third-party reviewer.
-   **Fact-Checking:** The manuscript successfully passes a technical fact-checking review by a qualified expert.

## Governance

This Constitution is the single source of truth for project principles and standards. It supersedes any conflicting guidance. Amendments require a documented proposal, review, and approval from the project lead. All development activities, including specification, planning, and implementation, MUST adhere to the principles outlined herein.

# --- Context7 Documentation Constitution ---

rule: Always use the Context7 MCP documentation server
description: The assistant must always query the Context7 MCP server to retrieve the most recent documentation before answering any question related to:

-   the project’s codebase
-   APIs
-   schema
-   architecture
-   Docusaurus content
-   CLI commands
-   configuration files

rule: Prioritize authoritative sources
description: When documentation exists both locally and on Context7 MCP, always treat the MCP version as the source of truth unless the user explicitly says otherwise.

rule: Automatic doc retrieval
description: When the user asks any project-related technical question, fetch the latest relevant documentation via the Context7 MCP server _before giving an answer_.

rule: Do not hallucinate
description: If no documentation exists in the MCP server for a requested topic, the assistant must state this clearly instead of fabricating content.

rule: Keep answers aligned with Docusaurus
description: When helping generate content for the Docusaurus book, always check the structure and content guidelines from the latest docs fetched via MCP to maintain consistency.

# --- End of Constitution ---

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
