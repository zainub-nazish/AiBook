/**
 * TypeScript interfaces for ChatWidget component
 */

/**
 * Source citation in API response
 */
export interface Source {
  title: string;
  url: string;
  score: number;
}

/**
 * A single message in the chat conversation
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
  sources?: Source[];
  isLoading?: boolean;
  isError?: boolean;
}

/**
 * API request payload for chat endpoint
 */
export interface QueryRequest {
  question: string;
  context?: string;
  session_id?: string;
  k?: number;
}

/**
 * API response from chat endpoint
 */
export interface QueryResponse {
  answer: string;
  sources: Source[];
  session_id: string;
  processing_time?: number;
}

/**
 * API error response
 */
export interface ErrorResponse {
  error: string;
  message: string;
  details?: string;
}

/**
 * Chat widget state
 */
export interface ChatState {
  isOpen: boolean;
  messages: ChatMessage[];
  sessionId: string;
  inputValue: string;
  isLoading: boolean;
  error: string | null;
  selectedContext: string | null;
}
