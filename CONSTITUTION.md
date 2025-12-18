# Physical AI & Humanoid Robotics Textbook - Constitution

## Project Overview
This is a comprehensive technical textbook for a university-level course on **Physical AI & Humanoid Robotics**. The textbook is built using **Docusaurus 3** with **TypeScript** and **Markdown (.md)** for all educational content.

---

## Project Rules & Standards

### 1. Technical Stack
- **Framework**: Docusaurus 3.9.2
- **Language**: TypeScript for configuration, Markdown (.md) for content
- **Node Version**: >=20.0
- **Writing Language**: Professional Technical English

### 2. Content Structure Requirements

#### All Chapters Must Include:
1. **Learning Objectives Section** - Clear, measurable learning outcomes at the beginning
2. **At least ONE of the following**:
   - Code block (Python, C++, ROS 2 launch files, YAML configs, etc.)
   - Diagram description (ASCII art, Mermaid diagrams, or detailed textual descriptions)
3. **Hands-on exercises or thought experiments**
4. **Summary/Key Takeaways section**

#### Content Tone & Style:
- ✅ Educational - Clear explanations with appropriate depth
- ✅ Encouraging - Motivate learners to experiment and explore
- ✅ Hands-on - Practical examples and exercises
- ✅ Professional - Industry-standard terminology and practices
- ❌ Avoid overly casual language
- ❌ Avoid unnecessary jargon without explanation

### 3. Technical Frameworks Focus

This textbook primarily covers:

1. **ROS 2 Humble** - Robot Operating System 2 (LTS version)
   - Node architecture
   - Publishers/Subscribers
   - Services and Actions
   - Parameters and Launch files
   - Navigation2 Stack

2. **Gazebo Classic & Gazebo Fortress** - Robot simulation
   - World creation
   - Robot model integration (URDF/SDF)
   - Sensor simulation
   - Physics engines

3. **NVIDIA Isaac Sim** - Advanced GPU-accelerated simulation
   - Synthetic data generation
   - Digital twins
   - Integration with ROS 2
   - Physics-based rendering

4. **OpenAI VLA (Vision-Language-Action) Models** - AI for robotics
   - Embodied AI concepts
   - Vision-language grounding
   - Action prediction from multimodal inputs
   - Integration with robot control systems

### 4. Directory Structure

```
humanoid-robotics-book/
├── docs/
│   ├── 00-introduction/
│   │   ├── 00-course-overview.md
│   │   ├── 01-what-is-physical-ai.md
│   │   └── 02-setup-environment.md
│   ├── 01-foundations/
│   │   ├── 00-robotics-fundamentals.md
│   │   ├── 01-kinematics-basics.md
│   │   ├── 02-dynamics-and-control.md
│   │   └── 03-sensors-and-actuators.md
│   ├── 02-ros2-essentials/
│   │   ├── 00-ros2-architecture.md
│   │   ├── 01-nodes-topics.md
│   │   ├── 02-services-actions.md
│   │   ├── 03-parameters-launch.md
│   │   └── 04-ros2-navigation.md
│   ├── 03-simulation/
│   │   ├── 00-simulation-overview.md
│   │   ├── 01-gazebo-basics.md
│   │   ├── 02-urdf-modeling.md
│   │   └── 03-isaac-sim-intro.md
│   ├── 04-perception/
│   │   ├── 00-vision-systems.md
│   │   ├── 01-depth-sensing.md
│   │   ├── 02-object-detection.md
│   │   └── 03-slam-localization.md
│   ├── 05-manipulation/
│   │   ├── 00-arm-kinematics.md
│   │   ├── 01-motion-planning.md
│   │   ├── 02-grasping-strategies.md
│   │   └── 03-force-control.md
│   ├── 06-locomotion/
│   │   ├── 00-bipedal-walking.md
│   │   ├── 01-balance-control.md
│   │   ├── 02-gait-generation.md
│   │   └── 03-terrain-adaptation.md
│   ├── 07-embodied-ai/
│   │   ├── 00-physical-ai-paradigm.md
│   │   ├── 01-vision-language-models.md
│   │   ├── 02-vla-architecture.md
│   │   ├── 03-action-prediction.md
│   │   └── 04-learning-from-demonstration.md
│   ├── 08-integration/
│   │   ├── 00-system-integration.md
│   │   ├── 01-behavior-trees.md
│   │   ├── 02-multi-modal-fusion.md
│   │   └── 03-real-world-deployment.md
│   └── 09-advanced-topics/
│       ├── 00-sim-to-real-transfer.md
│       ├── 01-safety-verification.md
│       ├── 02-human-robot-interaction.md
│       └── 03-future-perspectives.md
├── static/
│   ├── img/
│   │   ├── robots/
│   │   ├── diagrams/
│   │   └── screenshots/
│   └── files/
│       ├── code-examples/
│       └── datasets/
├── blog/ (Optional: Research updates, case studies)
└── src/ (Custom React components if needed)
```

### 5. Chapter Template

Every chapter should follow this template:

```markdown
---
sidebar_position: [number]
---

# [Chapter Title]

## Learning Objectives

By the end of this chapter, you will be able to:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Introduction

[Engaging introduction to the topic]

## [Section 1]

[Content with explanations]

### [Subsection if needed]

[Detailed content]

## Code Example / Diagram

[At least one code block or diagram description]

```python
# Example code
```

## Hands-On Exercise

[Practical exercise for students]

## Summary

[Key takeaways bullet points]

## Further Reading

[Optional: Additional resources]
```

### 6. Code Block Standards

- Use appropriate syntax highlighting (python, cpp, yaml, bash, xml)
- Include comments explaining non-obvious code
- Provide context before and after code blocks
- Test all code examples for correctness
- Include expected output where applicable

### 7. Diagram Standards

- Use Mermaid.js for flowcharts, sequence diagrams, and architecture diagrams
- Provide clear captions
- Ensure diagrams are readable in both light and dark modes
- Alternative: Provide detailed ASCII art or textual descriptions

### 8. Mathematical Notation

- Use LaTeX for mathematical expressions (Docusaurus supports KaTeX)
- Inline: `$equation$`
- Block: `$$equation$$`
- Always explain variables and notation

### 9. Accessibility & Inclusivity

- Use descriptive alt text for all images
- Maintain proper heading hierarchy
- Use clear, unambiguous language
- Provide multiple learning modalities (text, code, diagrams)

### 10. Version Control & Updates

- All chapters are versioned through Git
- Use semantic commit messages
- Document breaking changes in course structure
- Maintain a CHANGELOG.md

---

## Course Learning Path

### Prerequisites
- Basic programming (Python preferred)
- Linear algebra fundamentals
- Basic calculus
- Familiarity with Linux/Ubuntu

### Skill Progression
1. **Beginner** (Chapters 0-2): Foundations and ROS 2 basics
2. **Intermediate** (Chapters 3-6): Simulation, perception, manipulation, locomotion
3. **Advanced** (Chapters 7-9): Embodied AI, VLA models, system integration

---

## Build & Development Commands

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build

# Type checking
npm run typecheck

# Deploy (if configured)
npm run deploy
```

---

## Quality Assurance Checklist

Before publishing any chapter:
- [ ] Learning objectives clearly defined
- [ ] At least one code block or diagram present
- [ ] Hands-on exercise included
- [ ] Summary section complete
- [ ] No spelling or grammar errors
- [ ] Code examples tested
- [ ] Links verified
- [ ] Proper Markdown formatting
- [ ] Sidebar position set correctly

---

## Contribution Guidelines

When adding or modifying content:
1. Follow the chapter template
2. Maintain consistent terminology
3. Cross-reference related chapters
4. Update sidebars.ts if adding new files
5. Test builds locally before committing
6. Use professional technical English
7. Stay focused on the four main frameworks (ROS 2, Gazebo, Isaac Sim, VLA models)

---

## License & Attribution

[Specify license - e.g., MIT, CC BY-NC-SA 4.0]

All code examples should be ready to use in educational contexts.

---

## Contact & Support

[Instructor/Author contact information]
[Course website/repository]

---

**Last Updated**: 2025-12-17
**Docusaurus Version**: 3.9.2
**Target Audience**: University students (Junior/Senior level) and professionals entering humanoid robotics
