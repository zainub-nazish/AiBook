# Tasks: RAG Retrieval-Enabled Agent

**Input**: Design documents from `/specs/007-rag-retrieval-agent/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/cli-interface.md

**Tests**: Not explicitly requested - manual CLI testing only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project type**: Single file in existing backend folder
- **Source**: `backend/agent.py`
- **Dependencies**: Reuse from `backend/retrieve.py`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install new dependency and configure environment

- [x] T001 Install openai package in backend/.venv using pip install openai
- [x] T002 Add OPENAI_API_KEY to backend/.env file
- [x] T003 Verify all 4 env vars present (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY)

---

## Phase 2: Foundational (Core Agent Structure)

**Purpose**: Create the basic agent.py file structure that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create backend/agent.py with module docstring and imports
- [x] T005 Add constants (MODEL, MAX_QUERY_LENGTH, COLLECTION_NAME) in backend/agent.py
- [x] T006 Implement validate_env() function to check all 4 env vars in backend/agent.py
- [x] T007 Import retrieval functions from retrieve.py (get_qdrant_client, get_cohere_client, embed_query, search_similar)
- [x] T008 Initialize OpenAI client in backend/agent.py
- [x] T009 Define SYSTEM_PROMPT constant with grounding instructions in backend/agent.py
- [x] T010 Define tools list with retrieve_documentation function schema in backend/agent.py
- [x] T011 Implement retrieve_documentation() tool function in backend/agent.py
- [x] T012 Add argparse CLI setup with --k, --threshold, --debug, --no-citations flags in backend/agent.py
- [x] T013 Implement main() entry point with env validation in backend/agent.py

**Checkpoint**: Foundation ready - agent.py exists with core structure, imports work, env validated

---

## Phase 3: User Story 1 - Ask Question and Get Grounded Answer (Priority: P1) MVP

**Goal**: Enable users to ask questions and receive answers grounded in documentation with citations

**Independent Test**: Run `python agent.py "What is ROS 2?"` and verify answer includes source citations

### Implementation for User Story 1

- [x] T014 [US1] Implement process_query() function that handles single OpenAI API call in backend/agent.py
- [x] T015 [US1] Add tool call handling logic to process tool_calls from response in backend/agent.py
- [x] T016 [US1] Implement format_sources() to display citations (title, score, URL) in backend/agent.py
- [x] T017 [US1] Implement single_query_mode() for CLI one-shot queries in backend/agent.py
- [x] T018 [US1] Add --debug flag handling to show retrieved chunks and scores in backend/agent.py
- [x] T019 [US1] Add --no-citations flag handling to suppress source display in backend/agent.py
- [x] T020 [US1] Test single query: python agent.py "What is Isaac Sim?"

**Checkpoint**: User Story 1 complete - single questions work with grounded answers and citations

---

## Phase 4: User Story 2 - Handle Unknown Topics (Priority: P2)

**Goal**: Agent declines to answer when no relevant documentation found (anti-hallucination)

**Independent Test**: Run `python agent.py "What is the best pizza recipe?"` and verify agent declines

### Implementation for User Story 2

- [x] T021 [US2] Add relevance threshold check in process_query() for empty/low-score results in backend/agent.py
- [x] T022 [US2] Implement decline response format: "I couldn't find relevant information..." in backend/agent.py
- [x] T023 [US2] Update SYSTEM_PROMPT to emphasize ONLY answer from provided context in backend/agent.py
- [x] T024 [US2] Test out-of-scope query: python agent.py "What is the best pizza recipe?"
- [x] T025 [US2] Test low-relevance query: python agent.py "quantum computing algorithms"

**Checkpoint**: User Story 2 complete - agent correctly declines out-of-scope questions

---

## Phase 5: User Story 3 - Interactive Conversation (Priority: P3)

**Goal**: Multi-turn conversation with context maintained between questions

**Independent Test**: Start interactive mode, ask question, then ask follow-up referencing "it"

### Implementation for User Story 3

- [x] T026 [US3] Implement ConversationContext class to store message history in backend/agent.py
- [x] T027 [US3] Implement interactive_mode() with REPL loop in backend/agent.py
- [x] T028 [US3] Add welcome message and instructions display in interactive_mode() in backend/agent.py
- [x] T029 [US3] Handle special commands (exit, quit, clear, help) in interactive_mode() in backend/agent.py
- [x] T030 [US3] Pass conversation history to OpenAI API for context-aware responses in backend/agent.py
- [x] T031 [US3] Implement message history truncation (max 10 messages) in backend/agent.py
- [x] T032 [US3] Update main() to call interactive_mode() when no query argument provided in backend/agent.py
- [x] T033 [US3] Test interactive: run python agent.py, ask "What is ROS 2?", then "How do I install it?"

**Checkpoint**: User Story 3 complete - multi-turn conversation works with context retention

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, logging, and final validation

- [x] T034 Add graceful error handling for Qdrant connection failures in backend/agent.py
- [x] T035 Add graceful error handling for OpenAI API errors in backend/agent.py
- [x] T036 Add graceful error handling for Cohere embedding failures in backend/agent.py
- [x] T037 Implement logging for queries and responses using logging module in backend/agent.py
- [x] T038 Add sanitize_text() for Windows console compatibility (ASCII-safe output) in backend/agent.py
- [x] T039 Add exit codes (0=success, 1=config, 2=connection, 3=input, 4=runtime) in backend/agent.py
- [x] T040 Run quickstart.md validation: verify all examples work

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phase 3, 4, 5)**: All depend on Foundational (Phase 2) completion
  - Can proceed sequentially: P1 → P2 → P3
  - US1 must work before US2/US3 build on it
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Requires Phase 2 complete. No dependencies on other stories.
- **User Story 2 (P2)**: Requires US1 (process_query exists). Adds decline logic.
- **User Story 3 (P3)**: Requires US1 (process_query exists). Adds conversation loop.

### Within Each User Story

- Core implementation before integration
- Each story builds on previous but remains independently testable

### Parallel Opportunities (Limited)

This is a single-file implementation so parallelism is limited:
- T001, T002, T003 can run in parallel (env setup)
- Within each story, tasks are sequential (same file)

---

## Parallel Example: Setup Phase

```bash
# Launch all setup tasks together:
Task: "Install openai package"
Task: "Add OPENAI_API_KEY to .env"
Task: "Verify all 4 env vars"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T013)
3. Complete Phase 3: User Story 1 (T014-T020)
4. **STOP and VALIDATE**: Test `python agent.py "What is ROS 2?"`
5. MVP ready for demo

### Incremental Delivery

1. Setup + Foundational → Basic agent structure ready
2. Add User Story 1 → Single query works → MVP!
3. Add User Story 2 → Anti-hallucination works
4. Add User Story 3 → Full interactive conversation
5. Add Polish → Production ready

### Estimated Task Distribution

| Phase | Tasks | Description |
|-------|-------|-------------|
| Setup | 3 | T001-T003 |
| Foundational | 10 | T004-T013 |
| User Story 1 | 7 | T014-T020 |
| User Story 2 | 5 | T021-T025 |
| User Story 3 | 8 | T026-T033 |
| Polish | 7 | T034-T040 |
| **Total** | **40** | |

---

## Notes

- All tasks target single file: `backend/agent.py`
- [US1/US2/US3] labels map tasks to user stories for traceability
- Each user story has an independent test to verify completion
- Commit after each phase or logical group
- Stop at any checkpoint to validate story independently
- Windows console: Use ASCII-safe output (sanitize_text from retrieve.py)
