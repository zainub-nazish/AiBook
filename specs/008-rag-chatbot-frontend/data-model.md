# Data Model: Frontend-Backend Integration of RAG Chatbot

**Feature**: 008-rag-chatbot-frontend
**Date**: 2025-12-16

## Entity Definitions

### ChatMessage (Frontend)

A single message in the chat conversation.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| id | string | Yes | Unique message identifier | UUID format |
| role | string | Yes | "user" or "assistant" | One of allowed values |
| content | string | Yes | Message text content | Non-empty |
| timestamp | number | Yes | Unix timestamp in ms | Auto-set |
| sources | Source[] | No | Citations (assistant only) | Array of Source |
| isLoading | boolean | No | True while waiting for response | Default false |
| isError | boolean | No | True if error occurred | Default false |

### Source (Frontend/Backend)

A citation reference in an answer.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| title | string | Yes | Document/section title | Non-empty |
| url | string | Yes | Link to documentation page | Valid URL |
| score | number | Yes | Relevance score 0.0-1.0 | 0.0 <= score <= 1.0 |

### ChatState (Frontend)

React component state for chat widget.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| isOpen | boolean | Yes | Chat panel visibility | Default false |
| messages | ChatMessage[] | Yes | Conversation history | Array, may be empty |
| sessionId | string | Yes | Current session identifier | UUID, auto-generated |
| inputValue | string | Yes | Current input field value | May be empty |
| isLoading | boolean | Yes | Waiting for API response | Default false |
| error | string | No | Current error message | null if no error |
| selectedContext | string | No | Text selected from page | null if none |

### QueryRequest (Backend API)

Request body for chat endpoint.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| question | string | Yes | User's question text | Non-empty, max 8000 chars |
| context | string | No | Selected page text | Max 2000 chars |
| session_id | string | No | Session identifier | UUID format |
| k | integer | No | Number of chunks to retrieve | 1-20, default 5 |

### QueryResponse (Backend API)

Response body from chat endpoint.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| answer | string | Yes | AI-generated response | Non-empty |
| sources | Source[] | Yes | Source citations | Array, may be empty |
| session_id | string | Yes | Session identifier | UUID format |
| processing_time | number | No | Time taken in seconds | >= 0 |

### ErrorResponse (Backend API)

Error response from chat endpoint.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| error | string | Yes | Error type identifier | Non-empty |
| message | string | Yes | Human-readable error | Non-empty |
| details | string | No | Additional context | May be null |

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     FRONTEND (Docusaurus)                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ ChatWidget  │───►│  ChatState   │───►│   Messages    │  │
│  │  (click)    │    │  (useState)  │    │   (render)    │  │
│  └─────────────┘    └──────────────┘    └───────────────┘  │
│         │                  │                               │
│         │                  ▼                               │
│         │           ┌──────────────┐                       │
│         └──────────►│ QueryRequest │                       │
│                     │   (JSON)     │                       │
│                     └──────┬───────┘                       │
│                            │                               │
└────────────────────────────┼───────────────────────────────┘
                             │ POST /chat
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                     BACKEND (FastAPI)                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │  /chat      │───►│ process_query│───►│ retrieve_docs │  │
│  │  endpoint   │    │  (agent.py)  │    │ (retrieve.py) │  │
│  └─────────────┘    └──────────────┘    └───────────────┘  │
│         │                                      │           │
│         │                                      ▼           │
│         │                              ┌───────────────┐   │
│         │                              │    Qdrant     │   │
│         │                              │   (vectors)   │   │
│         │                              └───────────────┘   │
│         │                                                  │
│         ▼                                                  │
│  ┌──────────────┐                                          │
│  │QueryResponse │                                          │
│  │   (JSON)     │                                          │
│  └──────────────┘                                          │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

## Validation Rules

1. **Question text**:
   - Must not be empty or whitespace-only
   - Maximum 8000 characters
   - Trimmed before processing

2. **Context text**:
   - Optional (may be null/undefined)
   - Maximum 2000 characters if provided
   - Trimmed before processing

3. **Session ID**:
   - Auto-generated UUID if not provided
   - Used for conversation tracking
   - No server-side persistence (stateless API)

4. **Frontend state**:
   - Messages array preserved during session
   - Cleared on page refresh
   - Maximum 50 messages (truncate oldest)

## API Error Codes

| Error | HTTP Status | Message |
|-------|-------------|---------|
| EMPTY_QUESTION | 400 | Question text cannot be empty |
| QUESTION_TOO_LONG | 400 | Question exceeds maximum length |
| CONTEXT_TOO_LONG | 400 | Context exceeds maximum length |
| RETRIEVAL_ERROR | 500 | Failed to retrieve documentation |
| AGENT_ERROR | 500 | Failed to generate response |
| TIMEOUT | 504 | Request timed out |
