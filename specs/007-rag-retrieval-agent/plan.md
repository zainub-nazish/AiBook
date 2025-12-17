# Implementation Plan: RAG Retrieval-Enabled Agent

**Branch**: `007-rag-retrieval-agent` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/007-rag-retrieval-agent/spec.md`

## Summary

Build a conversational RAG agent using OpenAI Agents SDK that retrieves relevant documentation chunks from Qdrant and generates grounded answers with source citations. The agent will be a single-file CLI tool (`backend/agent.py`) that reuses existing retrieval logic from `retrieve.py`.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: openai (NEW), cohere (existing), qdrant-client (existing), python-dotenv (existing)
**Storage**: Qdrant Cloud (existing collection `rag_embedding`)
**Testing**: Manual CLI testing (pytest optional)
**Target Platform**: Windows/Linux/macOS CLI
**Project Type**: Single file in existing backend folder
**Performance Goals**: Response within 5 seconds
**Constraints**: Must only answer from retrieved content (no hallucination)
**Scale/Scope**: Local CLI tool, session-based conversation

## Constitution Check

*GATE: Pass - No violations. Project uses simple single-file approach.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | N/A | Single CLI tool, not a library |
| CLI Interface | PASS | CLI with stdin/stdout |
| Test-First | OPTIONAL | Manual testing for CLI agent |
| Simplicity | PASS | Single file, minimal dependencies |

## Project Structure

### Documentation (this feature)

```text
specs/007-rag-retrieval-agent/
├── plan.md              # This file
├── research.md          # Technology decisions (OpenAI Agents SDK)
├── data-model.md        # Entity definitions (Question, Answer, etc.)
├── quickstart.md        # Setup and usage guide
├── contracts/
│   └── cli-interface.md # CLI arguments and output format
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code

```text
backend/
├── .env                 # Environment variables (add OPENAI_API_KEY)
├── .venv/               # Virtual environment (existing)
├── main.py              # Embedding pipeline (existing)
├── retrieve.py          # Retrieval testing (existing - reuse functions)
└── agent.py             # NEW: RAG Agent implementation
```

**Structure Decision**: Single file `agent.py` in backend folder, following existing pattern of `main.py` and `retrieve.py`. Reuses functions from `retrieve.py` via import.

## Design Decisions

### D1: OpenAI Agents SDK for Agent Framework

**Decision**: Use `openai` Python package with function calling for custom retrieval tool
**Rationale**: Official SDK, simple API, good documentation
**Alternatives Rejected**: LangChain (too heavy), LlamaIndex (indexing focus), custom (more work)

### D2: Reuse retrieve.py Functions

**Decision**: Import `get_qdrant_client()`, `get_cohere_client()`, `embed_query()`, `search_similar()` from retrieve.py
**Rationale**: DRY principle, already tested and working
**Implementation**: `from retrieve import get_qdrant_client, ...`

### D3: In-Memory Conversation Context

**Decision**: Store conversation history as list of message dicts in memory
**Rationale**: Simple for CLI, no persistence needed, fits OpenAI message format
**Format**: `[{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]`

### D4: Grounding via System Prompt

**Decision**: System prompt instructs agent to ONLY use provided context, include citations
**Rationale**: Most reliable anti-hallucination approach for RAG
**Fallback**: If no relevant chunks, agent must respond "I couldn't find relevant information"

### D5: Retrieval as Tool Function

**Decision**: Create `retrieve_documentation(query: str)` as an OpenAI function tool
**Rationale**: Agent decides when/how to search, can refine queries
**Flow**: User question → Agent calls tool → Chunks returned → Agent generates answer

## Implementation Components

### Component 1: Environment Setup

- Add `openai` to dependencies
- Add `OPENAI_API_KEY` to .env
- Validate all 4 env vars on startup

### Component 2: Retrieval Tool

```python
def retrieve_documentation(query: str, k: int = 5) -> list[dict]:
    """Retrieve relevant documentation chunks from Qdrant.

    Returns list of chunks with text, url, title, score.
    """
    # Reuse from retrieve.py
    vector = embed_query(query, cohere_client)
    results = search_similar(qdrant_client, vector, k=k)
    return results
```

### Component 3: Agent Setup

```python
from openai import OpenAI

client = OpenAI()

SYSTEM_PROMPT = """You are a helpful assistant that answers questions about
Physical AI and Robotics. You MUST ONLY answer based on the context provided
by the retrieve_documentation tool. If no relevant information is found,
say "I couldn't find relevant information about that topic."

Always include source citations with URLs when answering."""

tools = [
    {
        "type": "function",
        "function": {
            "name": "retrieve_documentation",
            "description": "Search the documentation for relevant information",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {"type": "string", "description": "Search query"}
                },
                "required": ["query"]
            }
        }
    }
]
```

### Component 4: Conversation Loop

```python
def chat_loop():
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    while True:
        user_input = input("You: ").strip()
        if user_input.lower() in ["exit", "quit"]:
            break

        messages.append({"role": "user", "content": user_input})

        # Call OpenAI with tool
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            tools=tools,
            tool_choice="auto"
        )

        # Handle tool calls
        # ... process tool call, get results, continue conversation

        messages.append({"role": "assistant", "content": answer})
        print(f"\nAnswer:\n{answer}\n")
```

## Dependencies

| Package | Version | Purpose | Status |
|---------|---------|---------|--------|
| openai | >=1.0.0 | Agent LLM + function calling | NEW |
| cohere | >=5.0.0 | Query embeddings | Existing |
| qdrant-client | >=1.7.0 | Vector search | Existing |
| python-dotenv | >=1.0.0 | Environment config | Existing |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| OpenAI API costs | Use gpt-4o-mini (cheaper), limit conversation history |
| Hallucination | Strong system prompt, require citations |
| Slow response | Limit k=5 chunks, use fast model |
| Tool call errors | Graceful error handling, retry logic |

## Success Metrics

- [ ] Answer questions within 5 seconds (SC-001)
- [ ] Include accurate source citations (SC-002)
- [ ] Decline out-of-scope questions (SC-003)
- [ ] Maintain 3-turn conversation context (SC-004)
- [ ] Provide actionable error messages (SC-005)

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Run `/sp.implement` to execute tasks and create `agent.py`
3. Test with sample questions about the documentation
