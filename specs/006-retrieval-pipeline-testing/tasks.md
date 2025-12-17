# Tasks: Retrieval Pipeline Testing

**Input**: Design documents from `/specs/006-retrieval-pipeline-testing/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Not requested - manual CLI testing only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project structure**: `backend/` folder (existing infrastructure)
- **Target file**: `backend/retrieve.py` (single file implementation)

---

## Phase 1: Setup

**Purpose**: Create file structure and base imports

- [x] T001 Create `backend/retrieve.py` with module docstring and standard imports (os, time, logging, argparse)
- [x] T002 Add third-party imports (cohere, qdrant_client, dotenv) in `backend/retrieve.py`
- [x] T003 Configure logging with same format as `backend/main.py`

---

## Phase 2: Foundational (Core Infrastructure)

**Purpose**: Environment validation and client initialization - MUST complete before user stories

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Implement `validate_env()` function to check COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY in `backend/retrieve.py`
- [x] T005 Implement `get_qdrant_client()` function to initialize Qdrant client with retry logic in `backend/retrieve.py`
- [x] T006 Implement `get_cohere_client()` function to initialize Cohere client in `backend/retrieve.py`
- [x] T007 Define constants: COLLECTION_NAME="rag_embedding", DEFAULT_K=5, MODEL="embed-english-v3.0" in `backend/retrieve.py`

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Basic Similarity Query (Priority: P1) MVP

**Goal**: Developer can run a similarity query and receive relevant results with scores

**Independent Test**: Run `python retrieve.py "ROS 2 nodes"` and verify results are returned with scores, titles, URLs

### Implementation for User Story 1

- [x] T008 [US1] Implement `embed_query(text: str) -> list[float]` using Cohere with `input_type="search_query"` in `backend/retrieve.py`
- [x] T009 [US1] Implement `search_similar(query_vector: list, k: int, threshold: float) -> list[dict]` using Qdrant client in `backend/retrieve.py`
- [x] T010 [US1] Implement `format_result(result: dict, index: int) -> str` to display score, title, URL, text snippet in `backend/retrieve.py`
- [x] T011 [US1] Implement `display_results(results: list, query: str, elapsed: float)` to format full output in `backend/retrieve.py`
- [x] T012 [US1] Implement `search(query: str, k: int, threshold: float)` main search function in `backend/retrieve.py`
- [x] T013 [US1] Add CLI argument parsing with argparse: positional query, --k, --threshold in `backend/retrieve.py`
- [x] T014 [US1] Implement `main()` function to orchestrate search flow in `backend/retrieve.py`

**Checkpoint**: User Story 1 complete - can run `python retrieve.py "query"` and get results

---

## Phase 4: User Story 2 - Query Result Validation (Priority: P2)

**Goal**: Results include accurate metadata and handle edge cases properly

**Independent Test**: Run queries for known topics (e.g., "ROS 2") and verify results match expected documentation modules

### Implementation for User Story 2

- [x] T015 [US2] Add input validation in `search()`: empty query check, query length limit in `backend/retrieve.py`
- [x] T016 [US2] Add threshold filtering in `search_similar()` to exclude low-score results in `backend/retrieve.py`
- [x] T017 [US2] Handle empty results case with helpful message in `display_results()` in `backend/retrieve.py`
- [x] T018 [US2] Add text truncation in `format_result()` for long chunks (max 300 chars) in `backend/retrieve.py`

**Checkpoint**: User Story 2 complete - queries return validated, properly formatted results

---

## Phase 5: User Story 3 - Pipeline Health Check (Priority: P3)

**Goal**: Developer can run health check to verify all pipeline components are operational

**Independent Test**: Run `python retrieve.py --health` and verify connection status, collection info, vector count displayed

### Implementation for User Story 3

- [x] T019 [US3] Implement `check_qdrant_connection(client) -> tuple[bool, str]` in `backend/retrieve.py`
- [x] T020 [US3] Implement `check_collection_exists(client) -> tuple[bool, str, int]` returning exists, name, vector count in `backend/retrieve.py`
- [x] T021 [US3] Implement `run_sample_query(client) -> tuple[bool, float, int]` returning success, time_ms, result_count in `backend/retrieve.py`
- [x] T022 [US3] Implement `format_health_report(checks: dict) -> str` with table-style output in `backend/retrieve.py`
- [x] T023 [US3] Implement `health_check()` main function orchestrating all checks in `backend/retrieve.py`
- [x] T024 [US3] Add `--health` flag to argparse CLI in `backend/retrieve.py`
- [x] T025 [US3] Update `main()` to handle --health flag and call `health_check()` in `backend/retrieve.py`

**Checkpoint**: User Story 3 complete - can run `python retrieve.py --health` for full diagnostics

---

## Phase 6: Polish & Error Handling

**Purpose**: Improve error messages and edge case handling

- [x] T026 Add exit codes: 0=success, 1=config error, 2=connection error, 3=query error in `backend/retrieve.py`
- [x] T027 Add try/except blocks with actionable error messages per CLI contract in `backend/retrieve.py`
- [x] T028 Add query timing metrics (log elapsed time) in `backend/retrieve.py`
- [x] T029 Final testing: run quickstart.md test scenarios to validate all functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Core MVP
- **User Story 2 (Phase 4)**: Depends on US1 completion (extends search functionality)
- **User Story 3 (Phase 5)**: Depends on Foundational only (parallel with US1/US2 possible)
- **Polish (Phase 6)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Extends US1 search functionality - depends on US1
- **User Story 3 (P3)**: Independent health check - can run parallel with US1/US2 after Foundational

### Within Each User Story

- Core functions before orchestration
- Input validation before output formatting
- Error handling integrated throughout

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
- T004, T005, T006, T007 can all run in parallel (different functions)

**After Foundational**:
- US1 and US3 can run in parallel (different features)
- US2 must wait for US1 (extends same functions)

---

## Parallel Example: User Story 1

```bash
# Sequential within US1 (same file, function dependencies):
T008 → T009 → T010 → T011 → T012 → T013 → T014

# But US1 and US3 can run in parallel after Foundational:
# Developer A: T008-T014 (US1 - similarity search)
# Developer B: T019-T025 (US3 - health check)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T007)
3. Complete Phase 3: User Story 1 (T008-T014)
4. **STOP and VALIDATE**: Run `python retrieve.py "ROS 2"` - should return results
5. MVP ready for demo!

### Incremental Delivery

1. Setup + Foundational → Basic script structure
2. Add User Story 1 → Can query vectors (MVP!)
3. Add User Story 2 → Better validation and formatting
4. Add User Story 3 → Health check diagnostics
5. Add Polish → Production-ready error handling

### Single Developer Strategy

Execute in strict order: T001 → T002 → ... → T029

Commit after each phase completion:
- After T003: "feat: setup retrieve.py structure"
- After T007: "feat: add environment and client initialization"
- After T014: "feat: implement similarity search (US1)"
- After T018: "feat: add query validation (US2)"
- After T025: "feat: add health check (US3)"
- After T029: "feat: polish error handling"

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 29 |
| Phase 1 (Setup) | 3 tasks |
| Phase 2 (Foundational) | 4 tasks |
| Phase 3 (US1 - MVP) | 7 tasks |
| Phase 4 (US2) | 4 tasks |
| Phase 5 (US3) | 7 tasks |
| Phase 6 (Polish) | 4 tasks |
| MVP Scope | T001-T014 (14 tasks) |
| Parallel Opportunities | US1 + US3 after Foundational |

---

## Notes

- All tasks target single file: `backend/retrieve.py`
- No new dependencies required (reuses existing pyproject.toml)
- Tests not included (manual CLI testing per user request)
- Commit after each phase for clean git history
- Stop at any checkpoint to validate incrementally
