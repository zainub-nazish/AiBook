import React from 'react';
import styles from './styles.module.css';
import type { ChatMessage as ChatMessageType } from './types';

interface ChatMessageProps {
  message: ChatMessageType;
}

/**
 * Individual chat message component
 * Renders user messages, assistant messages, and loading states
 */
export function ChatMessage({ message }: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';

  // Loading state
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
        {/* Message content */}
        <div>{message.content}</div>

        {/* Source citations for assistant messages */}
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
                  {' '}({Math.round(source.score * 100)}% match)
                </span>
              </a>
            ))}
          </div>
        )}
      </div>

      {/* Timestamp */}
      <span className={styles.messageTimestamp}>
        {new Date(message.timestamp).toLocaleTimeString([], {
          hour: '2-digit',
          minute: '2-digit'
        })}
      </span>
    </div>
  );
}

export default ChatMessage;
