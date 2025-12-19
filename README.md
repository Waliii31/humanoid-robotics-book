# Physical AI & Humanoid Robotics Textbook

A comprehensive technical textbook for university-level courses on **Physical AI and Humanoid Robotics**, built with **Docusaurus 3** and **TypeScript**.

## 🎯 Overview

This textbook covers the intersection of artificial intelligence, robotics, and embodied cognition. Students will learn to build intelligent humanoid robots using cutting-edge technologies including:

- **ROS 2 Humble** - Industry-standard robot middleware
- **Gazebo & Isaac Sim** - Robot simulation platforms
- **OpenAI VLA Models** - Vision-Language-Action models for embodied AI
- **Modern robotics techniques** - Kinematics, dynamics, perception, control

## 📚 Course Structure

The textbook is organized into **10 comprehensive modules**:

### 0️⃣ Introduction
- Course overview and learning path
- What is Physical AI?
- Development environment setup

### 1️⃣ Foundations
- Robotics fundamentals
- Kinematics and dynamics
- Sensors and actuators
- Control systems

### 2️⃣ ROS 2 Essentials
- ROS 2 architecture and DDS
- Nodes, topics, services, actions
- Launch files and parameters
- Navigation stack

### 3️⃣ Simulation
- Gazebo fundamentals
- URDF robot modeling
- NVIDIA Isaac Sim
- Synthetic data generation

### 4️⃣ Perception
- Vision systems
- Depth sensing
- Object detection and tracking
- SLAM and localization

### 5️⃣ Manipulation
- Arm kinematics
- Motion planning (MoveIt2)
- Grasping strategies
- Force control

### 6️⃣ Locomotion
- Bipedal walking
- Balance and stability control
- Gait generation
- Terrain adaptation

### 7️⃣ Embodied AI
- Physical AI paradigm
- Vision-Language-Action (VLA) models
- Action prediction and control
- Learning from demonstration

### 8️⃣ Integration
- System integration
- Behavior trees
- Multi-modal sensor fusion
- Real-world deployment

### 9️⃣ Advanced Topics
- Sim-to-real transfer
- Safety and verification
- Human-robot interaction
- Future perspectives

## 🛠️ Technologies Used

- **Docusaurus 3.9.2** - Modern documentation framework
- **TypeScript** - Type-safe configuration
- **Markdown/MDX** - All educational content
- **Mermaid diagrams** - Interactive visualizations
- **Prism syntax highlighting** - Code examples in Python, C++, YAML, Bash
- **React** - Custom interactive components (optional)

## 📋 Prerequisites

Students should have:
- ✅ Intermediate Python programming skills
- ✅ Basic C++ understanding
- ✅ Linear algebra fundamentals
- ✅ Basic calculus
- ✅ Linux/Ubuntu familiarity

## 🚀 Getting Started

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/humanoid-robotics-book.git
cd humanoid-robotics-book

# Install dependencies
npm install
```

### Local Development

```bash
# Start development server (with hot reload)
npm start
```

This command starts a local development server at `http://localhost:3000` and opens in your browser. Most changes are reflected live without restarting the server.

### Build

```bash
# Generate static content
npm run build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

### Type Checking

```bash
# Run TypeScript type checking
npm run typecheck
```

## 📖 Content Guidelines

All chapters follow these requirements (see [CONSTITUTION.md](./CONSTITUTION.md) for details):

1. **Learning Objectives** - Clear, measurable outcomes
2. **Code Examples** - Tested, practical implementations
3. **Diagrams/Visuals** - At least one per chapter
4. **Hands-on Exercises** - Practical tasks for students
5. **Professional Tone** - Educational, encouraging, technical

### Chapter Template

Every chapter uses this structure:
```markdown
---
sidebar_position: [number]
---

# Chapter Title

## Learning Objectives
[List of objectives]

## Introduction
[Engaging introduction]

## [Main Content Sections]
[Detailed explanations with code]

## Hands-On Exercises
[Practical exercises]

## Summary
[Key takeaways]

## Further Reading
[Additional resources]
```

## 🤝 Contributing

We welcome contributions! To add or improve content:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-chapter`)
3. Follow the chapter template and constitution guidelines
4. Test your changes (`npm start`)
5. Submit a pull request

### Adding New Chapters

1. Create markdown file in appropriate module directory:
   ```
   docs/[module-name]/[chapter-number]-[chapter-name].md
   ```

2. Update `sidebars.ts` to include new chapter

3. Follow the quality assurance checklist in CONSTITUTION.md

## 📐 Project Structure

```
humanoid-robotics-book/
├── docs/                    # All educational content
│   ├── 00-introduction/
│   ├── 01-foundations/
│   ├── 02-ros2-essentials/
│   ├── ... (up to 09-advanced-topics)
├── blog/                    # Research updates, case studies
├── src/                     # Custom React components
│   ├── components/
│   ├── css/
│   └── pages/
├── static/                  # Static assets
│   ├── img/
│   └── files/
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Sidebar organization
├── package.json             # Dependencies
├── CONSTITUTION.md          # Project rules and standards
└── README.md                # This file
```

## 📜 License

[Specify your license - e.g., MIT, CC BY-NC-SA 4.0]

## 👥 Authors & Maintainers

[Your name and contact information]

## 🙏 Acknowledgments

- ROS 2 community
- NVIDIA Isaac Sim team
- OpenAI and Physical Intelligence teams
- Modern Robotics textbook authors (Lynch & Park)

## 📞 Support & Community

- **Issues**: [GitHub Issues](https://github.com/your-org/humanoid-robotics-book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-org/humanoid-robotics-book/discussions)
- **ROS Discourse**: [discourse.ros.org](https://discourse.ros.org)

## 🎓 Using This in Your Course

This textbook is designed for:
- **15-week semester course** (2-3 chapters per week)
- **Intensive workshop** (2-week intensive program)
- **Graduate seminar** (focus on chapters 7-9)
- **Self-paced learning** (8-10 hours per week recommended)

### Recommended Hardware for Students

**Minimum**: 
- CPU: 4 cores | RAM: 8GB | GPU: Integrated graphics

**Recommended**: 
- CPU: 8 cores | RAM: 16GB | GPU: NVIDIA RTX 3060+

See [Environment Setup](./docs/00-introduction/02-setup-environment.md) for detailed requirements.

---

**Built with ❤️ using Docusaurus** | **Empowering the next generation of roboticists** 🤖

























