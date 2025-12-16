# Research: Retrieval Pipeline Testing

**Feature**: 006-retrieval-pipeline-testing
**Date**: 2025-12-15

## Technical Context Analysis

### Existing Infrastructure

**Decision**: Reuse existing backend infrastructure from feature 005
**Rationale**: The embedding pipeline (`backend/main.py`) already has:
- Qdrant client connection setup
- Cohere embedding generation
- Environment variable handling
- Logging infrastructure
**Alternatives considered**: Creating a separate project - rejected because it would duplicate dependencies and configuration

### Embedding Model Consistency

**Decision**: Use same Cohere `embed-english-v3.0` model with `input_type="search_query"` for queries
**Rationale**:
- Indexing uses `input_type="search_document"`
- Cohere requires different input types for documents vs queries for optimal similarity
- Model must match (1024 dimensions) for vector compatibility
**Alternatives considered**: Using different embedding model - rejected because vectors would be incompatible

### Query Implementation Pattern

**Decision**: Create `retrieve.py` as standalone script in `backend/` folder
**Rationale**:
- Follows user's explicit request
- Maintains separation of concerns (indexing vs retrieval)
- Can be run independently for testing
- Shares same `.env` configuration
**Alternatives considered**: Adding retrieval functions to main.py - rejected because it mixes indexing and retrieval concerns

### Result Format

**Decision**: Display results in structured console output with:
- Similarity score (percentage)
- Document title
- Source URL
- Text snippet (truncated)
**Rationale**: Developers need quick visual validation; verbose output helps debugging
**Alternatives considered**: JSON output only - rejected as less readable for manual testing

### Health Check Pattern

**Decision**: Include health check function that validates:
1. Qdrant connectivity
2. Collection existence
3. Vector count
4. Sample query execution
**Rationale**: Single command for pipeline validation, useful for CI/CD
**Alternatives considered**: Separate health check script - rejected because it adds complexity

### Error Handling Strategy

**Decision**: Fail fast with clear error messages
**Rationale**: Testing tool should surface issues immediately, not silently continue
**Alternatives considered**: Best-effort approach - rejected because it could mask pipeline issues

## Dependencies Analysis

| Dependency | Version | Purpose | Already Installed |
|------------|---------|---------|-------------------|
| cohere | >=5.0.0 | Query embedding generation | Yes (main.py) |
| qdrant-client | >=1.7.0 | Vector similarity search | Yes (main.py) |
| python-dotenv | >=1.0.0 | Environment configuration | Yes (main.py) |

**No new dependencies required** - all dependencies already in `pyproject.toml`

## Technical Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| Which embedding input_type for queries? | `search_query` (Cohere requirement) |
| Where to place retrieve.py? | `backend/retrieve.py` (user specified) |
| How to format output? | Structured console output with scores |
| Default k value? | 5 (reasonable for testing) |
| Collection name? | `rag_embedding` (from main.py) |

## Integration Points

1. **Environment Variables**: Same `.env` file as main.py
2. **Collection**: Same `rag_embedding` collection
3. **Embedding Model**: Same `embed-english-v3.0`
4. **Vector Dimensions**: Same 1024 dimensions
