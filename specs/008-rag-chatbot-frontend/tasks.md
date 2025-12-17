# Tasks: Frontend-Backend Integration of RAG Chatbot

**Input**: Design documents from `/specs/008-rag-chatbot-frontend/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chat-api.md

**Tests**: Manual integration testing (no automated tests required per spec)

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` (Python, FastAPI)
- **Frontend**: `book/src/` (React, Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [x] T001 Install FastAPI and uvicorn in backend virtual environment: `backend/.venv/Scripts/pip.exe install fastapi uvicorn`
- [x] T002 [P] Verify Docusaurus React setup in book/package.json supports custom components
- [x] T003 [P] Create ChatWidget component directory structure: `book/src/components/ChatWidget/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user story implementation

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create FastAPI app skeleton with CORS middleware in backend/api.py
- [x] T005 [P] Create Pydantic models (QueryRequest, QueryResponse, ErrorResponse) in backend/api.py
- [x] T006 [P] Create Docusaurus theme wrapper in book/src/theme/Root.tsx
- [x] T007 Implement /health endpoint in backend/api.py for connectivity checks

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask Question via Chat Widget (Priority: P1)

**Goal**: Reader can click chat widget, type question, receive AI answer with source citations

**Independent Test**: Open Docusaurus site, click chat icon, type "What is ROS 2?", verify answer appears with sources

### Implementation for User Story 1

- [x] T008 [US1] Implement POST /chat endpoint in backend/api.py (import from agent.py)
- [x] T009 [US1] Add request validation (empty question, max length) in backend/api.py
- [x] T010 [US1] Add error handling and proper HTTP status codes in backend/api.py
- [x] T011 [P] [US1] Create ChatMessage TypeScript interface in book/src/components/ChatWidget/types.ts
- [x] T012 [P] [US1] Create Source TypeScript interface in book/src/components/ChatWidget/types.ts
- [x] T013 [US1] Create main ChatWidget component with open/close toggle in book/src/components/ChatWidget/index.tsx
- [x] T014 [US1] Add message input field with send button in book/src/components/ChatWidget/index.tsx
- [x] T015 [US1] Implement API call to POST /chat using fetch in book/src/components/ChatWidget/index.tsx
- [x] T016 [US1] Display AI response with formatted text in book/src/components/ChatWidget/index.tsx
- [x] T017 [US1] Display source citations as clickable links in book/src/components/ChatWidget/index.tsx
- [x] T018 [US1] Add loading indicator while waiting for response in book/src/components/ChatWidget/index.tsx
- [x] T019 [P] [US1] Create CSS styles for chat widget container in book/src/components/ChatWidget/styles.module.css
- [x] T020 [P] [US1] Create CSS styles for message bubbles in book/src/components/ChatWidget/styles.module.css
- [x] T021 [US1] Add error message display for API failures in book/src/components/ChatWidget/index.tsx
- [x] T022 [US1] Register ChatWidget in Root.tsx to appear on all pages in book/src/theme/Root.tsx

**Checkpoint**: User Story 1 complete - basic Q&A functionality works

---

## Phase 4: User Story 2 - Context-Aware Query from Page Content (Priority: P2)

**Goal**: Reader can select text on page and ask chatbot to explain it

**Independent Test**: Select text on documentation page, click "Ask about this", verify text included in query context

### Implementation for User Story 2

- [x] T023 [US2] Add context field handling in POST /chat endpoint in backend/api.py
- [x] T024 [US2] Modify agent query to include context in system prompt in backend/api.py
- [x] T025 [US2] Add text selection detection listener in book/src/components/ChatWidget/index.tsx
- [x] T026 [US2] Create "Ask about this" popup on text selection in book/src/components/ChatWidget/index.tsx
- [x] T027 [US2] Store selected text as context state in book/src/components/ChatWidget/index.tsx
- [x] T028 [US2] Include context in API request payload in book/src/components/ChatWidget/index.tsx
- [x] T029 [US2] Display context indicator in chat input area in book/src/components/ChatWidget/index.tsx
- [x] T030 [P] [US2] Add CSS styles for text selection popup in book/src/components/ChatWidget/styles.module.css
- [x] T031 [P] [US2] Add CSS styles for context indicator in book/src/components/ChatWidget/styles.module.css

**Checkpoint**: User Story 2 complete - context-aware queries work

---

## Phase 5: User Story 3 - View Chat History in Session (Priority: P3)

**Goal**: Reader can have multi-turn conversation with visible history

**Independent Test**: Ask multiple questions, verify both questions and answers visible, context maintained

### Implementation for User Story 3

- [x] T032 [US3] Add session_id handling in POST /chat endpoint in backend/api.py
- [x] T033 [US3] Implement message history state management in book/src/components/ChatWidget/index.tsx
- [x] T034 [US3] Create ChatMessage component for rendering individual messages in book/src/components/ChatWidget/ChatMessage.tsx
- [x] T035 [US3] Display scrollable message list with auto-scroll to bottom in book/src/components/ChatWidget/index.tsx
- [x] T036 [US3] Add clear history functionality (clear button or "clear" command) in book/src/components/ChatWidget/index.tsx
- [x] T037 [US3] Generate and maintain session_id across messages in book/src/components/ChatWidget/index.tsx
- [x] T038 [P] [US3] Add CSS styles for message history container in book/src/components/ChatWidget/styles.module.css
- [x] T039 [P] [US3] Add CSS styles for user vs assistant message differentiation in book/src/components/ChatWidget/styles.module.css

**Checkpoint**: User Story 3 complete - multi-turn conversation works

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Mobile responsiveness, edge cases, and final validation

- [x] T040 Add mobile responsive CSS (375px minimum width) in book/src/components/ChatWidget/styles.module.css
- [x] T041 [P] Add keyboard support (Enter to send) in book/src/components/ChatWidget/index.tsx
- [x] T042 [P] Add empty question prevention (disable send button) in book/src/components/ChatWidget/index.tsx
- [x] T043 Add request timeout handling (10 second limit) in book/src/components/ChatWidget/index.tsx
- [x] T044 Run manual end-to-end test per quickstart.md
- [x] T045 Verify all success criteria (SC-001 through SC-006) per spec.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2

### Within Each User Story

- Backend tasks before frontend tasks that call them
- TypeScript interfaces before components that use them
- CSS styles can run in parallel with component logic
- Core implementation before polish/edge cases

### Parallel Opportunities

**Phase 1 (Setup)**:
- T002 and T003 can run in parallel

**Phase 2 (Foundational)**:
- T005 and T006 can run in parallel

**User Story 1**:
- T011 and T012 (interfaces) can run in parallel
- T019 and T020 (CSS) can run in parallel

**User Story 2**:
- T030 and T031 (CSS) can run in parallel

**User Story 3**:
- T038 and T039 (CSS) can run in parallel

**Phase 6 (Polish)**:
- T041 and T042 can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch interfaces in parallel:
Task: "Create ChatMessage TypeScript interface in book/src/components/ChatWidget/types.ts"
Task: "Create Source TypeScript interface in book/src/components/ChatWidget/types.ts"

# Launch CSS in parallel:
Task: "Create CSS styles for chat widget container in book/src/components/ChatWidget/styles.module.css"
Task: "Create CSS styles for message bubbles in book/src/components/ChatWidget/styles.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (3 tasks)
2. Complete Phase 2: Foundational (4 tasks)
3. Complete Phase 3: User Story 1 (15 tasks)
4. **STOP and VALIDATE**: Test basic Q&A functionality
5. Deploy/demo if ready - MVP complete!

### Incremental Delivery

1. Setup + Foundational (7 tasks) → Foundation ready
2. Add User Story 1 (15 tasks) → Test → MVP deployed
3. Add User Story 2 (9 tasks) → Test → Context-aware queries
4. Add User Story 3 (8 tasks) → Test → Chat history
5. Add Polish (6 tasks) → Final validation

---

## Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| Phase 1: Setup | 3 | Dependencies and directory structure |
| Phase 2: Foundational | 4 | FastAPI skeleton, CORS, theme wrapper |
| Phase 3: US1 (P1) | 15 | Basic Q&A chat widget |
| Phase 4: US2 (P2) | 9 | Context-aware queries |
| Phase 5: US3 (P3) | 8 | Chat history |
| Phase 6: Polish | 6 | Mobile, edge cases, validation |
| **Total** | **45** | |

**Parallel Opportunities**: 12 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phases 1-3 (22 tasks) for basic Q&A functionality
