import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';
import type { ChatMessage, Source, QueryRequest, QueryResponse } from './types';

// API configuration - Hugging Face Spaces backend
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://zainubnazish-rag-chatbot.hf.space'
  : 'http://localhost:8000';

const REQUEST_TIMEOUT = 30000; // 30 seconds

/**
 * Generate a UUID for messages and sessions
 */
function generateId(): string {
  return crypto.randomUUID?.() ||
    'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
}

/**
 * Chat message bubble component
 */
function MessageBubble({ message }: { message: ChatMessage }): React.JSX.Element {
  const isUser = message.role === 'user';

  if (message.isLoading) {
    return (
      <div className={styles.loadingMessage}>
        <div className={styles.loadingDots}>
          <span></span>
          <span></span>
          <span></span>
        </div>
      </div>
    );
  }

  return (
    <div className={`${styles.messageWrapper} ${isUser ? styles.user : styles.assistant}`}>
      <div className={`${styles.message} ${
        isUser ? styles.userMessage :
        message.isError ? styles.errorMessage :
        styles.assistantMessage
      }`}>
        <div>{message.content}</div>
        {!isUser && message.sources && message.sources.length > 0 && (
          <div className={styles.sources}>
            <div className={styles.sourcesTitle}>Sources:</div>
            {message.sources.map((source, idx) => (
              <a
                key={idx}
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.sourceLink}
              >
                {source.title}
                <span className={styles.sourceScore}>
                  {' '}({Math.round(source.score * 100)}%)
                </span>
              </a>
            ))}
          </div>
        )}
      </div>
      <span className={styles.messageTimestamp}>
        {new Date(message.timestamp).toLocaleTimeString([], {
          hour: '2-digit',
          minute: '2-digit'
        })}
      </span>
    </div>
  );
}

/**
 * Main ChatWidget component
 */
export function ChatWidget(): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId] = useState(() => generateId());
  const [selectedContext, setSelectedContext] = useState<string | null>(null);
  const [selectionPopup, setSelectionPopup] = useState<{x: number; y: number; text: string} | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Text selection handler for context-aware queries
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 10 && selectedText.length < 2000) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectionPopup({
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
            text: selectedText
          });
        }
      } else {
        setSelectionPopup(null);
      }
    };

    const handleMouseDown = () => {
      setSelectionPopup(null);
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, []);

  /**
   * Send message to API
   */
  const sendMessage = useCallback(async () => {
    const question = inputValue.trim();
    if (!question || isLoading) return;

    // Clear input and error
    setInputValue('');
    setError(null);

    // Add user message
    const userMessage: ChatMessage = {
      id: generateId(),
      role: 'user',
      content: question,
      timestamp: Date.now()
    };
    setMessages(prev => [...prev, userMessage]);

    // Add loading message
    const loadingId = generateId();
    setMessages(prev => [...prev, {
      id: loadingId,
      role: 'assistant',
      content: '',
      timestamp: Date.now(),
      isLoading: true
    }]);
    setIsLoading(true);

    try {
      // Create request payload
      const payload: QueryRequest = {
        question,
        session_id: sessionId,
        k: 5
      };

      // Include context if selected
      if (selectedContext) {
        payload.context = selectedContext;
        setSelectedContext(null); // Clear after use
      }

      // Create abort controller for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT);

      const response = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail?.message || `Request failed: ${response.status}`);
      }

      const data: QueryResponse = await response.json();

      // Replace loading message with actual response
      setMessages(prev => prev.map(msg =>
        msg.id === loadingId
          ? {
              id: msg.id,
              role: 'assistant' as const,
              content: data.answer,
              timestamp: Date.now(),
              sources: data.sources
            }
          : msg
      ));

    } catch (err) {
      const errorMessage = err instanceof Error
        ? (err.name === 'AbortError'
            ? 'Request timed out. Please try again.'
            : err.message)
        : 'An unexpected error occurred';

      // Replace loading message with error
      setMessages(prev => prev.map(msg =>
        msg.id === loadingId
          ? {
              id: msg.id,
              role: 'assistant' as const,
              content: errorMessage,
              timestamp: Date.now(),
              isError: true
            }
          : msg
      ));

      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [inputValue, isLoading, sessionId, selectedContext]);

  /**
   * Handle keyboard events
   */
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  /**
   * Clear chat history
   */
  const clearHistory = () => {
    setMessages([]);
    setError(null);
  };

  /**
   * Handle "Ask about this" selection
   */
  const handleAskAboutSelection = () => {
    if (selectionPopup) {
      setSelectedContext(selectionPopup.text);
      setSelectionPopup(null);
      setIsOpen(true);
      // Clear the text selection
      window.getSelection()?.removeAllRanges();
    }
  };

  return (
    <div className={styles.chatWidget}>
      {/* Selection Popup */}
      {selectionPopup && (
        <div
          className={styles.selectionPopup}
          style={{
            left: selectionPopup.x,
            top: selectionPopup.y,
            transform: 'translate(-50%, -100%)'
          }}
          onClick={handleAskAboutSelection}
        >
          Ask about this
        </div>
      )}

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3 className={styles.chatTitle}>Ask about the docs</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              <svg viewBox="0 0 24 24" width="20" height="20" fill="currentColor">
                <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/>
              </svg>
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <div className={styles.welcomeTitle}>Welcome!</div>
                <div className={styles.welcomeText}>
                  Ask me anything about Physical AI, Robotics, ROS 2, Isaac Sim, and more.
                  I'll search the documentation to find relevant answers.
                </div>
              </div>
            ) : (
              messages.map(message => (
                <MessageBubble key={message.id} message={message} />
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Clear History */}
          {messages.length > 0 && (
            <div className={styles.clearHistory}>
              <button
                className={styles.clearHistoryButton}
                onClick={clearHistory}
              >
                Clear history
              </button>
            </div>
          )}

          {/* Context Indicator */}
          {selectedContext && (
            <div className={styles.contextIndicator}>
              <span className={styles.contextText}>
                Context: "{selectedContext.substring(0, 50)}..."
              </span>
              <button
                className={styles.clearContext}
                onClick={() => setSelectedContext(null)}
              >
                Clear
              </button>
            </div>
          )}

          {/* Input Area */}
          <div className={styles.inputArea}>
            <input
              ref={inputRef}
              type="text"
              className={styles.input}
              placeholder="Ask a question..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
              <svg viewBox="0 0 24 24" fill="currentColor">
                <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"/>
              </svg>
            </button>
          </div>
        </div>
      )}

      {/* Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <svg viewBox="0 0 24 24" fill="currentColor">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/>
          </svg>
        ) : (
          <svg viewBox="0 0 24 24" fill="currentColor">
            <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z"/>
          </svg>
        )}
      </button>
    </div>
  );
}

export default ChatWidget;
