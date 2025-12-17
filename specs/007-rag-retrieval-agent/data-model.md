# Data Model: RAG Retrieval-Enabled Agent

**Feature**: 007-rag-retrieval-agent
**Date**: 2025-12-16

## Entity Definitions

### Question

User's natural language query to the RAG agent.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| text | string | Yes | The question text from user | Non-empty, max 8000 chars |
| conversation_id | string | No | Session identifier | Auto-generated UUID |
| timestamp | datetime | Yes | When question was asked | Auto-set |

**Relationships**:
- Has one Answer (response)
- Belongs to ConversationContext (session)

### RetrievalResult

Document chunks retrieved from Qdrant based on semantic similarity.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| text | string | Yes | Chunk content | Non-empty |
| url | string | Yes | Source documentation URL | Valid URL format |
| title | string | Yes | Document/section title | Non-empty |
| score | float | Yes | Similarity score 0.0-1.0 | 0.0 <= score <= 1.0 |
| chunk_index | int | No | Position in original doc | >= 0 |

**Relationships**:
- Many-to-one with Question (multiple chunks per query)
- Referenced by Answer (for citations)

### Answer

Agent's response to a user question.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| text | string | Yes | The answer content | Non-empty |
| citations | list[Citation] | Yes | Source references used | May be empty if declined |
| confidence | string | Yes | "high", "medium", "low", "none" | One of allowed values |
| declined | bool | Yes | True if no relevant info found | Default false |
| timestamp | datetime | Yes | When answer was generated | Auto-set |

**Relationships**:
- One-to-one with Question
- Contains multiple Citations

### Citation

Reference to source material used in an answer.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| url | string | Yes | Source documentation URL | Valid URL |
| title | string | Yes | Document title | Non-empty |
| relevance_score | float | Yes | How relevant this source was | 0.0-1.0 |

### ConversationContext

Session history for multi-turn conversations.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| session_id | string | Yes | Unique session identifier | UUID format |
| messages | list[Message] | Yes | History of Q&A exchanges | Ordered by timestamp |
| created_at | datetime | Yes | Session start time | Auto-set |
| max_history | int | No | Max messages to retain | Default 10 |

**State Transitions**:
- ACTIVE: Accepting new questions
- EXPIRED: Session timed out (future feature)

### Message

Single entry in conversation history.

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| role | string | Yes | "user" or "assistant" | One of allowed values |
| content | string | Yes | Message text | Non-empty |
| timestamp | datetime | Yes | When message was created | Auto-set |

## Data Flow

```
┌─────────────────┐
│    Question     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐     ┌─────────────────┐
│  Query Embed    │────►│  Qdrant Search  │
└─────────────────┘     └────────┬────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │RetrievalResults │
                        │  (list of N)    │
                        └────────┬────────┘
                                 │
                                 ▼
┌─────────────────┐     ┌─────────────────┐
│ Conversation    │────►│   OpenAI Agent  │
│    Context      │     │  (with context) │
└─────────────────┘     └────────┬────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │     Answer      │
                        │ (with citations)│
                        └─────────────────┘
```

## Validation Rules

1. **Question text**:
   - Must not be empty or whitespace-only
   - Maximum 8000 characters (same as retrieve.py)
   - No special preprocessing required

2. **RetrievalResult score**:
   - Scores below 0.3 indicate low relevance
   - Scores above 0.5 indicate high relevance
   - No results = agent should decline to answer

3. **Answer confidence mapping**:
   - "high": Multiple results with score > 0.5
   - "medium": At least one result with score > 0.3
   - "low": Only results with score < 0.3
   - "none": No results found → declined = true

4. **Conversation context**:
   - Default to keeping last 10 messages (5 Q&A pairs)
   - Truncate older messages to stay within token limits
