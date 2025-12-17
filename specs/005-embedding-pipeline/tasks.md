# Tasks: Embedding Pipeline Setup

**Input**: Design documents from `/specs/005-embedding-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested - skipping test tasks
**Organization**: Tasks grouped by user story for independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
```
backend/
├── main.py              # Single file with all functions
├── .env.example         # Environment variables template
├── pyproject.toml       # UV project configuration
└── tests/
    └── test_main.py     # Unit tests (optional)
```

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create backend folder and initialize UV project

- [x] T001 Create backend/ directory at repository root
- [x] T002 Initialize UV project with `uv init` in backend/
- [x] T003 Add dependencies to backend/pyproject.toml: cohere, qdrant-client, requests, beautifulsoup4, python-dotenv
- [x] T004 [P] Create backend/.env.example with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, BASE_URL placeholders
- [x] T005 [P] Create backend/main.py with imports and logging configuration

**Checkpoint**: Project structure ready, dependencies installed

---

## Phase 2: Foundational (Shared Infrastructure)

**Purpose**: Core utilities needed by all user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Implement environment loading in backend/main.py using python-dotenv
- [x] T007 Add validate_credentials() function in backend/main.py to verify API keys are present
- [x] T008 Configure logging format in backend/main.py: `%(asctime)s - %(levelname)s - %(message)s`
- [x] T009 Add generate_point_id(url, chunk_index) helper function in backend/main.py for deterministic UUIDs

**Checkpoint**: Foundation ready - user story implementation can begin

---

## Phase 3: User Story 1 - URL Crawling and Text Extraction (Priority: P1) 🎯 MVP

**Goal**: Crawl Docusaurus site and extract clean text from all documentation pages

**Independent Test**: Run `get_all_urls()` and `extract_text_from_url()` on https://ai-book-mu.vercel.app/ and verify clean text output

### Implementation for User Story 1

- [x] T010 [US1] Implement get_all_urls(base_url) function in backend/main.py - parse sitemap.xml from https://ai-book-mu.vercel.app/sitemap.xml
- [x] T011 [US1] Add sitemap XML parsing logic using requests and xml.etree in backend/main.py
- [x] T012 [US1] Implement extract_text_from_url(url) function in backend/main.py - fetch HTML and extract text
- [x] T013 [US1] Add BeautifulSoup parsing to extract_text_from_url() - remove nav, footer, header, scripts
- [x] T014 [US1] Extract page title from HTML <title> or <h1> tag in extract_text_from_url()
- [x] T015 [US1] Preserve code block content during text extraction (identify <pre> and <code> tags)
- [x] T016 [US1] Add error handling for invalid/unreachable URLs with logging in backend/main.py
- [x] T017 [US1] Implement chunk_text(text, chunk_size=1500, overlap=200) function in backend/main.py

**Checkpoint**: User Story 1 complete - can crawl site and get clean text chunks

---

## Phase 4: User Story 2 - Cohere Embedding Generation (Priority: P2)

**Goal**: Generate vector embeddings from text chunks using Cohere API

**Independent Test**: Pass sample text chunks to `embed()` and verify 1024-dimensional vectors returned

### Implementation for User Story 2

- [x] T018 [US2] Implement embed(texts, api_key) function in backend/main.py using Cohere SDK
- [x] T019 [US2] Configure Cohere client with model="embed-english-v3.0" and input_type="search_document"
- [x] T020 [US2] Add batching logic to embed() - batch up to 96 texts per API call
- [x] T021 [US2] Implement exponential backoff retry for Cohere rate limits (1s, 2s, 4s, max 3 retries)
- [x] T022 [US2] Add handling for empty/whitespace text chunks - skip with warning logged
- [x] T023 [US2] Log embedding progress: batch number, texts processed, success/failure

**Checkpoint**: User Story 2 complete - can generate embeddings from text

---

## Phase 5: User Story 3 - Qdrant Vector Storage (Priority: P3)

**Goal**: Store embeddings in Qdrant with metadata for RAG retrieval

**Independent Test**: Store sample vectors in Qdrant and verify retrieval with correct metadata

### Implementation for User Story 3

- [x] T024 [US3] Implement create_collection(client, collection_name="rag_embedding") in backend/main.py
- [x] T025 [US3] Configure collection with vector size=1024 and distance=Cosine
- [x] T026 [US3] Add collection existence check - create only if not exists
- [x] T027 [US3] Implement save_chunk_to_qdrant(client, collection_name, vectors, payloads) in backend/main.py
- [x] T028 [US3] Use upsert operation for save_chunk_to_qdrant() to support re-indexing
- [x] T029 [US3] Include payload: url, title, chunk_index, text in each Qdrant point
- [x] T030 [US3] Add Qdrant connection error handling with retry and logging
- [x] T031 [US3] Log storage progress: vectors stored, batch success/failure

**Checkpoint**: User Story 3 complete - embeddings can be stored and retrieved

---

## Phase 6: User Story 4 - End-to-End Pipeline Execution (Priority: P4)

**Goal**: Orchestrate complete pipeline with single command execution

**Independent Test**: Run `uv run main.py` and verify vectors appear in Qdrant collection

### Implementation for User Story 4

- [x] T032 [US4] Implement main() function in backend/main.py to orchestrate full pipeline
- [x] T033 [US4] Add pipeline flow: validate_credentials → get_all_urls → loop(extract → chunk → embed → save)
- [x] T034 [US4] Initialize Qdrant client with environment variables in main()
- [x] T035 [US4] Call create_collection() at pipeline start
- [x] T036 [US4] Add progress logging: pages processed, chunks created, vectors stored
- [x] T037 [US4] Implement summary logging at pipeline end: total pages, chunks, vectors, duration
- [x] T038 [US4] Add if __name__ == "__main__": main() block in backend/main.py

**Checkpoint**: User Story 4 complete - full pipeline runs with single command

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling improvements and documentation

- [x] T039 [P] Add comprehensive docstrings to all functions in backend/main.py
- [x] T040 [P] Update backend/.env.example with comments explaining each variable
- [x] T041 Verify pipeline runs successfully against https://ai-book-mu.vercel.app/
- [x] T042 Run quickstart.md validation - verify documented steps work

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - blocks all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - MVP
- **User Story 2 (Phase 4)**: Depends on Foundational - can run parallel to US1
- **User Story 3 (Phase 5)**: Depends on Foundational - can run parallel to US1/US2
- **User Story 4 (Phase 6)**: Depends on US1, US2, US3 - integration phase
- **Polish (Phase 7)**: Depends on all user stories complete

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US1 (Crawling) | Foundational | US2, US3 |
| US2 (Embedding) | Foundational | US1, US3 |
| US3 (Storage) | Foundational | US1, US2 |
| US4 (Pipeline) | US1, US2, US3 | None |

### Within Each User Story

Since all functions are in single file (main.py), sequential execution recommended:
- Core function implementation first
- Error handling second
- Logging third

### Parallel Opportunities

```text
# Phase 1 parallel tasks:
T004 (.env.example) || T005 (main.py skeleton)

# After Foundational, these story implementations can overlap:
US1 (T010-T017) || US2 (T018-T023) || US3 (T024-T031)
# Note: All modify main.py - coordinate to avoid conflicts

# Phase 7 parallel:
T039 (docstrings) || T040 (.env.example update)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T009)
3. Complete Phase 3: User Story 1 (T010-T017)
4. **VALIDATE**: Test crawling and text extraction independently
5. Can demo text extraction without embeddings

### Incremental Delivery

1. Setup + Foundational → Project ready
2. Add US1 → Text extraction works → Demo crawling
3. Add US2 → Embeddings work → Demo embedding generation
4. Add US3 → Storage works → Demo Qdrant storage
5. Add US4 → Full pipeline → Demo end-to-end

### Recommended Execution Order

```text
T001 → T002 → T003 → T004+T005 (parallel)
→ T006 → T007 → T008 → T009
→ T010 → T011 → T012 → T013 → T014 → T015 → T016 → T017
→ T018 → T019 → T020 → T021 → T022 → T023
→ T024 → T025 → T026 → T027 → T028 → T029 → T030 → T031
→ T032 → T033 → T034 → T035 → T036 → T037 → T038
→ T039+T040 (parallel) → T041 → T042
```

---

## Summary

| Phase | Task Count | Description |
|-------|------------|-------------|
| Setup | 5 | Project initialization |
| Foundational | 4 | Core utilities |
| US1 (P1) | 8 | URL crawling & text extraction |
| US2 (P2) | 6 | Cohere embedding generation |
| US3 (P3) | 8 | Qdrant vector storage |
| US4 (P4) | 7 | End-to-end pipeline |
| Polish | 4 | Documentation & validation |
| **Total** | **42** | |

---

## Notes

- All implementation in single file: backend/main.py
- [P] tasks can run in parallel if no file conflicts
- [US#] label maps task to user story for traceability
- Commit after each completed function
- Target site: https://ai-book-mu.vercel.app/
- Sitemap: https://ai-book-mu.vercel.app/sitemap.xml
