# Implementation Plan: Embedding Pipeline Setup

**Branch**: `005-embedding-pipeline` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-embedding-pipeline/spec.md`

## Summary

Build a Python-based embedding pipeline that crawls the deployed Docusaurus site (https://ai-book-mu.vercel.app/), siteMap URL (https://ai-book-mu.vercel.app/sitemap.xml), extracts clean text, generates embeddings via Cohere API, and stores vectors in Qdrant for RAG retrieval. Implementation uses UV package manager and consists of a single `main.py` file with modular functions.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: UV (package manager), cohere, qdrant-client, requests, beautifulsoup4
**Storage**: Qdrant vector database (cloud or local)
**Testing**: pytest (for unit tests)
**Target Platform**: Linux/Windows/macOS CLI
**Project Type**: Single backend project
**Performance Goals**: Index 100 pages in under 10 minutes
**Constraints**: Cohere API rate limits, Qdrant connection availability
**Scale/Scope**: Single Docusaurus site (~20-50 pages initially)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Library-First | PASS | Single file design per user request, can be extracted to library later |
| CLI Interface | PASS | main.py runs as CLI script |
| Test-First | PASS | Will create tests in tasks phase |
| Observability | PASS | Logging included in design |
| Simplicity | PASS | Single file, minimal dependencies, no over-engineering |

## Project Structure

### Documentation (this feature)

```text
specs/005-embedding-pipeline/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Single file with all pipeline functions
├── .env.example         # Environment variables template
├── pyproject.toml       # UV/Python project configuration
└── tests/
    └── test_main.py     # Unit tests for pipeline functions
```

**Structure Decision**: Single backend folder with main.py containing all functions as specified by user:
- `get_all_urls()` - Discover all doc pages from sitemap/crawling
- `extract_text_from_url()` - Fetch and clean HTML content
- `chunk_text()` - Split text into embedding-sized chunks
- `embed()` - Generate embeddings via Cohere API
- `create_collection()` - Create Qdrant collection named "rag_embedding"
- `save_chunk_to_qdrant()` - Upsert vectors with metadata
- `main()` - Orchestrate the pipeline

## Complexity Tracking

No violations requiring justification. Design follows YAGNI principle with single-file approach.
