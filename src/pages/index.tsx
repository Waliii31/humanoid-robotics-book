import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Feature Card Component
interface FeatureCardProps {
  title: string;
  icon: string;
  description: string;
  gradient: string;
}

function FeatureCard({ title, icon, description, gradient }: FeatureCardProps) {
  return (
    <div className={styles.featureCard} style={{ background: gradient }}>
      <div className={styles.featureIcon}>{icon}</div>
      <Heading as="h3" className={styles.featureTitle}>
        {title}
      </Heading>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroOverlay}></div>
      <div className={styles.heroContent}>
        <div className={styles.heroGrid}></div>
        <Heading as="h1" className={styles.heroTitle}>
          {siteConfig.title}
        </Heading>
        <p className={styles.heroSubtitle}>
          Bridging the Digital Brain and Physical Body
        </p>
        <p className={styles.heroDescription}>
          Master the convergence of AI, robotics, and embodied intelligence through
          hands-on simulation, ROS 2 development, and real-world deployment strategies.
        </p>
        <div className={styles.heroButtons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/week01-02-physical-ai/intro">
            🚀 Quick Start - Week 1
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            📚 View Curriculum
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features: FeatureCardProps[] = [
    {
      title: 'Sim-to-Real',
      icon: '🎮',
      description: 'Master NVIDIA Isaac Sim and Gazebo for photorealistic robot simulation, physics-based training, and seamless reality transfer.',
      gradient: 'linear-gradient(135deg, rgba(62, 139, 255, 0.15) 0%, rgba(62, 139, 255, 0.05) 100%)',
    },
    {
      title: 'The Nervous System',
      icon: '🧠',
      description: 'Build distributed robotic systems with ROS 2, from sensor fusion and navigation to real-time control architectures.',
      gradient: 'linear-gradient(135deg, rgba(255, 107, 53, 0.15) 0%, rgba(255, 107, 53, 0.05) 100%)',
    },
    {
      title: 'Embodied AI',
      icon: '🤖',
      description: 'Develop humanoid robots with advanced perception, natural language interaction, and physical intelligence capabilities.',
      gradient: 'linear-gradient(135deg, rgba(92, 159, 255, 0.15) 0%, rgba(92, 159, 255, 0.05) 100%)',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={styles.featuresTitle}>
          Core Pillars of Physical AI
        </Heading>
        <div className={styles.featureGrid}>
          {features.map((props, idx) => (
            <FeatureCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStack() {
  return (
    <section className={styles.techStack}>
      <div className="container">
        <Heading as="h2" className={styles.techStackTitle}>
          Industry-Standard Technologies
        </Heading>
        <div className={styles.techGrid}>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>NVIDIA Isaac</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>ROS 2</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>PyTorch</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>Gazebo</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>Docker</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techBadge}>Python</span>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive technical textbook for Physical AI and Humanoid Robotics, covering simulation, ROS 2, and embodied intelligence.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <TechStack />
      </main>
    </Layout>
  );
}

