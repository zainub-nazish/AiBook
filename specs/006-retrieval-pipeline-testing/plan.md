# Implementation Plan: Retrieval Pipeline Testing

**Branch**: `006-retrieval-pipeline-testing` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-retrieval-pipeline-testing/spec.md`

## Summary

Create `retrieve.py` in the `backend/` folder to test the RAG retrieval pipeline. The script will connect to Qdrant, execute similarity searches using Cohere embeddings, and provide health check functionality to validate the end-to-end pipeline.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere>=5.0.0, qdrant-client>=1.7.0, python-dotenv>=1.0.0 (all existing)
**Storage**: Qdrant Cloud (existing collection: `rag_embedding`)
**Testing**: Manual CLI testing + pytest (optional)
**Target Platform**: Local development (Windows/Linux/Mac)
**Project Type**: Single backend project (CLI tool)
**Performance Goals**: Query results within 2 seconds
**Constraints**: Must use same embedding model as indexing (Cohere embed-english-v3.0)
**Scale/Scope**: Developer testing tool, not production service

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | PASS | Reuses existing backend infrastructure |
| CLI Interface | PASS | Provides text in/out via command line |
| Test-First | N/A | Testing tool itself; manual validation |
| Simplicity | PASS | Single file, minimal dependencies |

## Project Structure

### Documentation (this feature)

```text
specs/006-retrieval-pipeline-testing/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technical research
├── data-model.md        # Data entities
├── quickstart.md        # Usage guide
├── contracts/
│   └── cli-interface.md # CLI contract
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Existing embedding pipeline
├── retrieve.py          # NEW: Retrieval testing script
├── pyproject.toml       # Dependencies (no changes needed)
├── .env                 # Environment configuration
└── .env.example         # Environment template
```

**Structure Decision**: Single file `retrieve.py` in existing `backend/` folder. No new directories needed - reuses existing infrastructure.

## Implementation Approach

### Phase 1: Core Retrieval Function

1. Create `retrieve.py` with Qdrant connection setup (reuse pattern from main.py)
2. Implement `embed_query()` using Cohere with `input_type="search_query"`
3. Implement `search_similar()` using Qdrant client
4. Format and display results

### Phase 2: CLI Interface

1. Add argparse for command-line arguments
2. Support `--k` and `--threshold` parameters
3. Add `--health` flag for health check mode

### Phase 3: Health Check

1. Implement connection validation
2. Implement collection check with vector count
3. Implement sample query test
4. Format health report output

### Phase 4: Error Handling

1. Validate environment variables
2. Handle connection errors gracefully
3. Handle empty/invalid queries
4. Provide actionable error messages

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| File location | `backend/retrieve.py` | User specified; follows existing structure |
| Embedding input_type | `search_query` | Cohere best practice for query vs document |
| Default k | 5 | Reasonable for quick testing |
| Output format | Structured console | Readable for developers |
| Collection name | `rag_embedding` | Matches main.py |

## Dependencies

No new dependencies required. All needed packages already in `pyproject.toml`:
- cohere>=5.0.0
- qdrant-client>=1.7.0
- python-dotenv>=1.0.0

## Complexity Tracking

No constitution violations. Single-file implementation with minimal complexity.

## Artifacts Generated

| Artifact | Path | Status |
|----------|------|--------|
| Research | specs/006-retrieval-pipeline-testing/research.md | Complete |
| Data Model | specs/006-retrieval-pipeline-testing/data-model.md | Complete |
| CLI Contract | specs/006-retrieval-pipeline-testing/contracts/cli-interface.md | Complete |
| Quickstart | specs/006-retrieval-pipeline-testing/quickstart.md | Complete |
| Plan | specs/006-retrieval-pipeline-testing/plan.md | Complete |

## Next Steps

Run `/sp.tasks` to generate implementation tasks for `retrieve.py`.
