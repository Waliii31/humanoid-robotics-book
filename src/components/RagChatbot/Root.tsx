import React, { useState, useEffect } from 'react';
import RagChatbot from './RagChatbot';
import './RagChatbot.css';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [hasScrolled, setHasScrolled] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 100) {
        setHasScrolled(true);
      } else {
        setHasScrolled(false);
      }
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div className="floating-chatbot">
      {isOpen ? (
        <div className="chatbot-container">
          <RagChatbot />
          <button
            className="close-chatbot-button"
            onClick={() => setIsOpen(false)}
            aria-label="Close chatbot"
          >
            ×
          </button>
        </div>
      ) : (
        <button
          className={`floating-chatbot-button ${hasScrolled ? 'scrolled' : ''}`}
          onClick={() => setIsOpen(true)}
          aria-label="Open documentation assistant"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM8 16.5L12 14L16 16.5L12 19L8 16.5ZM9.5 11C8.67 11 8 10.33 8 9.5C8 8.67 8.67 8 9.5 8C10.33 8 11 8.67 11 9.5C11 10.33 10.33 11 9.5 11ZM14.5 11C13.67 11 13 10.33 13 9.5C13 8.67 13.67 8 14.5 8C15.33 8 16 8.67 16 9.5C16 10.33 15.33 11 14.5 11Z"
              fill="currentColor"
            />
          </svg>
        </button>
      )}
    </div>
  );
};

export default FloatingChatbot;