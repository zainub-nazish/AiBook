# Research: Frontend-Backend Integration of RAG Chatbot

**Feature**: 008-rag-chatbot-frontend
**Date**: 2025-12-16

## Technical Context Analysis

### FastAPI Backend for Chat API

**Decision**: Create new `api.py` in backend folder with FastAPI chat endpoint
**Rationale**:
- FastAPI provides automatic OpenAPI docs
- Easy CORS configuration
- Async support for non-blocking requests
- Reuse existing agent.py logic via import
**Alternatives considered**:
- Flask: Less async support, no automatic docs
- Django: Overkill for simple API
- Extend agent.py directly: Mixes CLI and API concerns

### React Chat Component in Docusaurus

**Decision**: Create custom React component in `book/src/components/ChatWidget/`
**Rationale**:
- Docusaurus is built on React
- Custom components are standard pattern
- Can use React hooks for state management
- CSS modules for scoped styling
**Alternatives considered**:
- Third-party chat widget: Less customizable
- iframe embedding: Poor UX, cross-origin issues
- Web Components: Less React integration

### API Communication Pattern

**Decision**: Use fetch API with async/await for REST calls
**Rationale**:
- Native browser API, no extra dependencies
- Works with Docusaurus build
- Simple error handling
**Alternatives considered**:
- Axios: Extra dependency not needed
- React Query: Overkill for single endpoint
- WebSocket: Not needed for request-response pattern

### Chat Widget Positioning

**Decision**: Fixed position bottom-right corner with expandable panel
**Rationale**:
- Standard chat widget pattern users recognize
- Doesn't interfere with documentation content
- Easy to dismiss/minimize
**Alternatives considered**:
- Sidebar: Takes up reading space
- Modal: Blocks content
- Inline: Disrupts page flow

### State Management

**Decision**: React useState/useContext for local state
**Rationale**:
- Simple for session-based chat
- No external dependencies
- Easy to reset on page refresh
**Alternatives considered**:
- Redux: Overkill for chat state
- Zustand: Extra dependency
- localStorage: Not needed (session-only)

## Dependencies

### Backend (FastAPI)

| Dependency | Version | Purpose | New? |
|------------|---------|---------|------|
| fastapi | >=0.100.0 | REST API framework | YES |
| uvicorn | >=0.23.0 | ASGI server | YES |
| pydantic | >=2.0.0 | Request/response models | No (existing) |
| openai | >=1.0.0 | Agent LLM | No (existing) |
| cohere | >=5.0.0 | Query embeddings | No (existing) |
| qdrant-client | >=1.7.0 | Vector search | No (existing) |

### Frontend (Docusaurus/React)

| Dependency | Version | Purpose | New? |
|------------|---------|---------|------|
| react | 18.x | UI framework | No (Docusaurus) |
| @docusaurus/core | 3.x | Documentation framework | No (existing) |

**No new frontend dependencies needed** - using native browser fetch and React hooks.

## Environment Variables

| Variable | Purpose | New? |
|----------|---------|------|
| COHERE_API_KEY | Query embeddings | No |
| QDRANT_URL | Vector database | No |
| QDRANT_API_KEY | Qdrant auth | No |
| OPENAI_API_KEY | Agent LLM | No |
| API_HOST | Backend URL for frontend | YES (config) |

## Technical Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| How to add chat to Docusaurus? | Custom React component in src/components/ |
| How to handle CORS? | FastAPI CORS middleware |
| Where to put FastAPI code? | New backend/api.py file |
| How to maintain chat state? | React useState, session-based |
| How to style chat widget? | CSS modules in component folder |

## Integration Points

### Backend Integration

1. **New file**: `backend/api.py`
   - Import from agent.py: `process_query`, `SYSTEM_PROMPT`, etc.
   - FastAPI app with `/chat` endpoint
   - CORS middleware for frontend origin
   - Pydantic models for request/response

2. **Endpoint**: `POST /chat`
   - Request: `{ question: string, context?: string, session_id?: string }`
   - Response: `{ answer: string, sources: [{title, url, score}], session_id: string }`

### Frontend Integration

1. **New files**:
   - `book/src/components/ChatWidget/index.tsx`
   - `book/src/components/ChatWidget/styles.module.css`

2. **Docusaurus integration**:
   - Add to `book/src/theme/Root.tsx` or wrap in layout
   - Available on all documentation pages

## File Structure

```text
backend/
├── .env                 # Environment variables
├── agent.py             # Existing RAG agent (reuse)
├── retrieve.py          # Existing retrieval (reuse)
└── api.py               # NEW: FastAPI server

book/
├── src/
│   ├── components/
│   │   └── ChatWidget/  # NEW: Chat component
│   │       ├── index.tsx
│   │       └── styles.module.css
│   └── theme/
│       └── Root.tsx     # NEW: Wrap app with chat
└── docusaurus.config.js # Config (no changes needed)
```
