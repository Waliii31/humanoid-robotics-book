import React, { type ReactNode } from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function Footer(): ReactNode {
  return (
    <footer className={styles.footer}>
      <div className={styles.footerContainer}>
        <div className={styles.footerContent}>
          <div className={styles.footerSection}>
            <h3 className={styles.footerTitle}>Physical AI & Humanoid Robotics</h3>
            <p className={styles.footerDescription}>
              A comprehensive technical textbook for embodied intelligence
            </p>
          </div>

          <div className={styles.footerSection}>
            <h4 className={styles.footerHeading}>Quick Links</h4>
            <ul className={styles.footerLinks}>
              <li>
                <Link to="/docs/intro" className={styles.footerLink}>
                  📚 Curriculum
                </Link>
              </li>
              <li>
                <Link to="/blog" className={styles.footerLink}>
                  📝 Research & Updates
                </Link>
              </li>
              <li>
                <a
                  href="https://www.panaversity.com"
                  target="_blank"
                  rel="noopener noreferrer"
                  className={styles.footerLink}>
                  🎓 Panaversity
                </a>
              </li>
            </ul>
          </div>

          <div className={styles.footerSection}>
            <h4 className={styles.footerHeading}>Resources</h4>
            <ul className={styles.footerLinks}>
              <li>
                <a
                  href="https://github.com/your-org/humanoid-robotics-book"
                  target="_blank"
                  rel="noopener noreferrer"
                  className={styles.footerLink}>
                  💻 GitHub Repository
                </a>
              </li>
              <li>
                <Link to="/docs/resources/hardware" className={styles.footerLink}>
                  🔧 Hardware Requirements
                </Link>
              </li>
            </ul>
          </div>
        </div>

        <div className={styles.footerBottom}>
          <div className={styles.footerDivider}></div>
          <p className={styles.footerCopyright}>
            © {new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook.
            Built with <span className={styles.footerHeart}>❤️</span> using Docusaurus.
          </p>
          <p className={styles.footerPowered}>
            Powered by <a href="https://www.panaversity.com" target="_blank" rel="noopener noreferrer" className={styles.footerPanaversity}>Panaversity</a>
          </p>
        </div>
      </div>
    </footer>
  );
}

export default React.memo(Footer);

