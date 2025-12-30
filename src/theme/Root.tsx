import React from 'react';
import FloatingChatbot from '../components/RagChatbot/Root';

// Docusaurus Root component - this will be rendered once for the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
}