# Implementation Plan: Frontend-Backend Integration of RAG Chatbot

**Branch**: `008-rag-chatbot-frontend` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/008-rag-chatbot-frontend/spec.md`

## Summary

Integrate the RAG backend (feature 007) with the Docusaurus frontend by creating a FastAPI REST API endpoint and a React chat widget component. The chat widget will be embedded in all documentation pages, allowing readers to ask questions and receive AI-generated answers with source citations.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React (frontend)
**Primary Dependencies**: FastAPI, uvicorn (backend NEW), React (frontend existing)
**Storage**: Qdrant Cloud (existing), Browser sessionStorage (frontend)
**Testing**: Manual integration testing
**Target Platform**: Web browser (desktop + mobile)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: Response within 10 seconds
**Constraints**: CORS enabled, JSON over HTTP, session-based chat
**Scale/Scope**: Local development, single concurrent user

## Constitution Check

*GATE: Pass - No violations. Web app pattern is appropriate.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | N/A | Web app, not a library |
| CLI Interface | PASS | Backend has existing CLI (agent.py) |
| Test-First | OPTIONAL | Manual integration testing |
| Simplicity | PASS | Minimal new dependencies |

## Project Structure

### Documentation (this feature)

```text
specs/008-rag-chatbot-frontend/
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup and testing guide
├── contracts/
│   └── chat-api.md      # REST API contract
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code

```text
backend/
├── .env                 # Environment variables (existing)
├── agent.py             # RAG agent logic (existing - reuse)
├── retrieve.py          # Retrieval functions (existing - reuse)
└── api.py               # NEW: FastAPI server

book/
├── src/
│   ├── components/
│   │   └── ChatWidget/  # NEW: React chat component
│   │       ├── index.tsx
│   │       ├── ChatMessage.tsx
│   │       └── styles.module.css
│   └── theme/
│       └── Root.tsx     # NEW: Docusaurus theme wrapper
├── docusaurus.config.js # Existing config
└── package.json         # Existing dependencies
```

**Structure Decision**: Web application pattern with existing backend folder and Docusaurus in book folder. FastAPI serves API, React handles UI.

## Design Decisions

### D1: FastAPI for REST API

**Decision**: Create new `api.py` with FastAPI framework
**Rationale**:
- Automatic OpenAPI/Swagger docs
- Built-in request validation with Pydantic
- Easy CORS configuration
- Async support
**Alternatives Rejected**: Flask (no async), Django (overkill), Express.js (different language)

### D2: Reuse agent.py Logic

**Decision**: Import core functions from agent.py instead of duplicating
**Rationale**: DRY principle, already tested logic
**Implementation**: `from agent import process_query, SYSTEM_PROMPT`

### D3: React Chat Widget

**Decision**: Custom React component in Docusaurus src/components
**Rationale**:
- Docusaurus is React-based
- Full control over UI/UX
- No external dependencies
**Alternatives Rejected**: Third-party chat library (less customizable), iframe (CORS issues)

### D4: Fixed Bottom-Right Position

**Decision**: Chat widget fixed to bottom-right corner
**Rationale**: Standard pattern users recognize, doesn't block content
**Alternatives Rejected**: Sidebar (takes reading space), modal (blocks content)

### D5: Session-Based State

**Decision**: Use React useState, clear on page refresh
**Rationale**: Simple, matches spec requirement (no persistence)
**Alternatives Rejected**: localStorage (persistence not needed), Redux (overkill)

## Implementation Components

### Component 1: FastAPI Backend (api.py)

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI(title="RAG Chatbot API")

# CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://ai-book-mu.vercel.app"],
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    question: str
    context: str | None = None
    session_id: str | None = None
    k: int = 5

@app.post("/chat")
async def chat(request: QueryRequest):
    # Use existing agent logic
    ...
```

### Component 2: React Chat Widget

```tsx
// book/src/components/ChatWidget/index.tsx
export function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const sendMessage = async () => {
    // POST to /chat endpoint
    const response = await fetch('http://localhost:8000/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question: input }),
    });
    // Handle response...
  };

  return (
    <div className={styles.chatWidget}>
      {/* Chat UI */}
    </div>
  );
}
```

### Component 3: Docusaurus Integration

```tsx
// book/src/theme/Root.tsx
import { ChatWidget } from '../components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

## Dependencies

### Backend (NEW)

| Package | Version | Purpose |
|---------|---------|---------|
| fastapi | >=0.100.0 | REST API framework |
| uvicorn | >=0.23.0 | ASGI server |

### Frontend (No new dependencies)

Using existing React from Docusaurus.

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| CORS blocking requests | Configure FastAPI CORS middleware |
| Slow API responses | Show loading indicator, set timeout |
| Mobile responsiveness | Use CSS media queries, test on mobile |
| Browser compatibility | Use fetch API (widely supported) |

## API Contract Summary

### POST /chat

**Request**:
```json
{
  "question": "What is ROS 2?",
  "context": "optional selected text",
  "session_id": "uuid",
  "k": 5
}
```

**Response**:
```json
{
  "answer": "ROS 2 is...",
  "sources": [
    { "title": "...", "url": "...", "score": 0.85 }
  ],
  "session_id": "uuid"
}
```

## Success Metrics

- [ ] Chat widget appears on all documentation pages (SC-002)
- [ ] User can submit question and receive answer within 10 seconds (SC-001)
- [ ] Source citations are clickable links (FR-005)
- [ ] Mobile responsive at 375px width (SC-004)
- [ ] Error messages display on API failure (FR-007)

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Run `/sp.implement` to create api.py and ChatWidget
3. Test end-to-end integration locally
