# Research: RAG Retrieval-Enabled Agent

**Feature**: 007-rag-retrieval-agent
**Date**: 2025-12-16

## Technical Context Analysis

### OpenAI Agents SDK

**Decision**: Use `openai-agents` Python SDK for agent implementation
**Rationale**:
- Official SDK from OpenAI for building agents with tools
- Supports custom function tools (retrieval)
- Built-in conversation management
- Simple async/sync patterns
**Alternatives considered**:
- LangChain: Heavier dependency, more complex
- LlamaIndex: More focused on indexing than agents
- Custom implementation: More work, less maintained

### Retrieval Integration Pattern

**Decision**: Create custom retrieval tool function that wraps existing `retrieve.py` logic
**Rationale**:
- Reuse existing Qdrant connection code from `retrieve.py`
- Cohere embeddings already working
- Tool function returns context for agent to use
**Alternatives considered**:
- Direct Qdrant plugin: Not available in OpenAI Agents SDK
- Embedding in prompt: Less flexible, harder to control

### Agent Architecture

**Decision**: Single-file `agent.py` in `backend/` folder (per user request)
**Rationale**:
- User explicitly requested `agent.py` in backend folder
- Follows same pattern as `main.py` and `retrieve.py`
- Shares environment configuration
**Alternatives considered**:
- Separate agent package: Overkill for CLI tool
- Modular approach: User wants single file

### Grounding Strategy

**Decision**: Pass retrieval chunks to agent with explicit instruction to only use provided context
**Rationale**:
- System prompt instructs agent to ONLY answer from provided context
- Include source URLs in context for citation
- If no relevant context found, agent must decline
**Alternatives considered**:
- Post-processing filter: Harder to implement reliably
- Confidence scoring: Complex, unreliable

### Conversation Context

**Decision**: Maintain conversation history in memory (list of messages)
**Rationale**:
- Simple for CLI tool
- No persistence needed (session-based)
- Pass history to agent on each turn
**Alternatives considered**:
- File-based persistence: Overkill for MVP
- Database storage: Not needed for local CLI

## Dependencies

| Dependency | Version | Purpose | New? |
|------------|---------|---------|------|
| openai | >=1.0.0 | OpenAI Agents SDK | YES - needs to add |
| cohere | >=5.0.0 | Query embedding | No (existing) |
| qdrant-client | >=1.7.0 | Vector search | No (existing) |
| python-dotenv | >=1.0.0 | Environment | No (existing) |

**New dependency required**: `openai` package for Agents SDK

### Environment Variables

| Variable | Purpose | New? |
|----------|---------|------|
| COHERE_API_KEY | Query embeddings | No |
| QDRANT_URL | Vector database | No |
| QDRANT_API_KEY | Qdrant auth | No |
| OPENAI_API_KEY | Agent LLM | YES - needs to add |

**New env variable required**: `OPENAI_API_KEY`

## Technical Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| Which agent SDK? | openai-agents (official SDK) |
| Where to place file? | `backend/agent.py` (user specified) |
| How to integrate retrieval? | Custom tool function wrapping retrieve.py logic |
| How to prevent hallucination? | System prompt + only use provided context |
| How to handle conversation? | In-memory message history |

## Integration Points

1. **Reuse from retrieve.py**:
   - `get_qdrant_client()` - Qdrant connection
   - `get_cohere_client()` - Cohere for embeddings
   - `embed_query()` - Generate query vector
   - `search_similar()` - Qdrant similarity search

2. **New Components**:
   - OpenAI client initialization
   - Agent with custom retrieval tool
   - System prompt for grounding
   - Conversation loop with history
