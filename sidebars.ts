import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // 13-Week Humanoid Robotics Curriculum
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '📄 Getting Started Guide',
    },
    {
      type: 'category',
      label: '📚 Resources',
      collapsed: true,
      items: [
        'resources/hardware-requirements',
      ],
    },
    
    {
      type: 'category',
      label: '📘 Weeks 1-2: Physical AI & Embodied Intelligence',
      collapsed: false,
      items: [
        'week01-02-physical-ai/foundations-of-physical-ai',
        'week01-02-physical-ai/embodied-intelligence-architecture',
      ],
    },
    {
      type: 'category',
      label: '🤖 Weeks 3-5: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'week03-05-ros2-fundamentals/ros2-ecosystem-and-nodes',
        'week03-05-ros2-fundamentals/topics-services-actions',
        'week03-05-ros2-fundamentals/parameters-launch-tf2',
      ],
    },
    {
      type: 'category',
      label: '🎮 Weeks 6-7: Simulation with Gazebo & URDF',
      collapsed: false,
      items: [
        'week06-07-simulation/urdf-modeling-gazebo-basics',
        'week06-07-simulation/gazebo-ros-integration-control',
      ],
    },
    {
      type: 'category',
      label: '🎬 Weeks 8-10: NVIDIA Isaac Platform',
      collapsed: false,
      items: [
        'week08-10-nvidia-isaac/isaac-sim-introduction',
        'week08-10-nvidia-isaac/vslam-and-perception',
        'week08-10-nvidia-isaac/advanced-isaac-features',
      ],
    },
    {
      type: 'category',
      label: '🚶 Weeks 11-12: Humanoid Development',
      collapsed: false,
      items: [
        'week11-12-humanoid-dev/bipedal-locomotion-fundamentals',
        'week11-12-humanoid-dev/kinematics-whole-body-control',
      ],
    },
    {
      type: 'category',
      label: '🗣️ Week 13: Conversational Robotics',
      collapsed: false,
      items: [
        'week13-conversational-robotics/speech-to-action-whisper-gpt',
      ],
    },
  ],
};

export default sidebars;
