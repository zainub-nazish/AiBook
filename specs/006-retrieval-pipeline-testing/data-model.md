# Data Model: Retrieval Pipeline Testing

**Feature**: 006-retrieval-pipeline-testing
**Date**: 2025-12-15

## Entities

### Query

Represents a similarity search request.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| text | string | The search query text | Required, non-empty, max 8000 chars |
| k | integer | Number of results to return | Optional, default 5, range 1-100 |
| threshold | float | Minimum similarity score | Optional, default 0.0, range 0.0-1.0 |

### SearchResult

Represents a single result from similarity search.

| Field | Type | Description | Source |
|-------|------|-------------|--------|
| score | float | Cosine similarity score | Qdrant search response |
| text | string | Original document chunk | Qdrant payload.text |
| url | string | Source document URL | Qdrant payload.url |
| title | string | Document title | Qdrant payload.title |
| chunk_index | integer | Position of chunk in document | Qdrant payload.chunk_index |

### HealthReport

Represents the status of the RAG pipeline.

| Field | Type | Description |
|-------|------|-------------|
| qdrant_connected | boolean | True if Qdrant is reachable |
| collection_exists | boolean | True if rag_embedding collection exists |
| vector_count | integer | Number of vectors in collection |
| sample_query_success | boolean | True if sample query returned results |
| sample_query_time_ms | float | Time taken for sample query |
| errors | list[string] | List of any errors encountered |

## Existing Data (from Embedding Pipeline)

The retrieval feature reads from vectors stored by `main.py`:

### Qdrant Collection: `rag_embedding`

| Property | Value |
|----------|-------|
| Vector Size | 1024 (Cohere embed-english-v3.0) |
| Distance Metric | Cosine |
| Point ID | UUID5 from URL + chunk_index |

### Vector Payload Schema

```json
{
  "url": "https://ai-book-mu.vercel.app/docs/module-01-ros2/chapter-01",
  "title": "Introduction to ROS 2",
  "chunk_index": 0,
  "text": "ROS 2 (Robot Operating System 2) is the second generation..."
}
```

## Data Flow

```
┌─────────────────┐
│   User Query    │
│  "ROS 2 nodes"  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Cohere Embed   │
│  search_query   │
└────────┬────────┘
         │ 1024-dim vector
         ▼
┌─────────────────┐
│  Qdrant Search  │
│  rag_embedding  │
└────────┬────────┘
         │ top-k results
         ▼
┌─────────────────┐
│  SearchResult[] │
│  with payloads  │
└─────────────────┘
```

## Validation Rules

1. **Query text**: Must be non-empty after trimming whitespace
2. **K parameter**: Must be positive integer, capped at 100 to prevent excessive results
3. **Threshold**: Must be between 0.0 and 1.0 (inclusive)
4. **Results ordering**: Always sorted by score descending (highest similarity first)
