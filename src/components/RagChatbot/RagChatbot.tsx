import React, { useState, useRef, useEffect } from 'react';
import './RagChatbot.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

interface ChatResponse {
  response: string;
  retrieved_chunks: Array<{
    id: string;
    content: string;
    url: string;
    heading?: string;
    score?: number;
  }>;
  tokens_used: {
    prompt_tokens: number;
    completion_tokens: number;
    total_tokens: number;
  };
}

interface RagChatbotProps {
  backendUrl?: string;
}

const RagChatbot: React.FC<RagChatbotProps> = ({ backendUrl = '/api' }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [showSelectedTextMode, setShowSelectedTextMode] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Scroll to bottom of messages when new messages are added
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Function to get selected text from the page
  const handleTextSelection = () => {
    const selectedText = window.getSelection()?.toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
      setShowSelectedTextMode(true);
      setInputValue('');
    }
  };

  // Function to clear selected text mode
  const clearSelectedTextMode = () => {
    setSelectedText(null);
    setShowSelectedTextMode(false);
    setInputValue('');
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() && !selectedText) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue || (selectedText ? `About: ${selectedText.substring(0, 100)}...` : ''),
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Prepare the request body
      const requestBody = {
        messages: [
          {
            role: 'user',
            content: inputValue || `Question about: ${selectedText}`,
          }
        ],
        top_k: 5,
        temperature: 0.1,
        user_selected_text: selectedText || undefined,
      };

      // Determine the correct backend URL based on environment
      // For development, use localhost:8000
      // For production, you can modify this to use your deployed backend URL
      let apiUrl = backendUrl;

      if (!apiUrl || apiUrl === '/api') {
        // Default to localhost for development
        apiUrl = 'http://localhost:8000';
      }

      const response = await fetch(`${apiUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status} ${response.statusText}`);
      }

      const data: ChatResponse = await response.json();

      const botMessage: Message = {
        id: Date.now().toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);

      // Clear selected text mode after successful query
      if (selectedText) {
        clearSelectedTextMode();
      }
    } catch (err) {
      console.error('Error fetching chat response:', err);
      setError(err instanceof Error ? err.message : 'An unknown error occurred');

      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Documentation Assistant</h3>
        <div className="header-actions">
          <button
            className={`mode-toggle ${showSelectedTextMode ? 'active' : ''}`}
            onClick={() => setShowSelectedTextMode(!showSelectedTextMode)}
            title={showSelectedTextMode ? "Switch to full mode" : "Switch to selected text mode"}
          >
            {showSelectedTextMode ? "📄 Text Mode" : "🔍 Select Text"}
          </button>
          <button
            className="select-text-btn"
            onClick={handleTextSelection}
            disabled={showSelectedTextMode}
            title="Select text on the page to ask about it"
          >
            Select Text
          </button>
        </div>
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <div className="selected-text-content">
            <strong>Selected text:</strong> "{selectedText.substring(0, 150)}{selectedText.length > 150 ? '...' : ''}"
          </div>
          <button
            className="clear-selection-btn"
            onClick={clearSelectedTextMode}
            title="Clear selected text"
          >
            ✕
          </button>
        </div>
      )}

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="empty-state">
            <p>Ask a question about the documentation!</p>
            {showSelectedTextMode && (
              <p className="mode-instruction">
                Selected text mode: Ask questions about the highlighted text on the page
              </p>
            )}
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.role}`}
            >
              <div className="message-content">
                {message.content}
              </div>
              <div className="message-timestamp">
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message assistant">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="error-message">
          Error: {error}
        </div>
      )}

      <form className="chat-input-form" onSubmit={handleSubmit}>
        {showSelectedTextMode && !selectedText && (
          <p className="input-instruction">
            Please select text on the page first, or type a general question
          </p>
        )}

        <div className="input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={
              showSelectedTextMode && selectedText
                ? "Ask a question about the selected text..."
                : "Ask a question about the documentation..."
            }
            disabled={isLoading}
            rows={3}
          />
          <button
            type="submit"
            disabled={isLoading || (!inputValue.trim() && !selectedText)}
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </form>
    </div>
  );
};

export default RagChatbot;