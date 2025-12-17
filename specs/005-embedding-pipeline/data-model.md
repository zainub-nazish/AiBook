# Data Model: Embedding Pipeline

**Feature**: 005-embedding-pipeline
**Date**: 2025-12-15

## Entities

### 1. Document

Represents a single documentation page from the Docusaurus site.

| Field | Type | Description |
|-------|------|-------------|
| url | string | Full URL of the documentation page |
| title | string | Page title extracted from HTML |
| text | string | Clean text content (HTML stripped) |

**Validation Rules**:
- URL must be valid HTTP/HTTPS URL
- URL must be within the base domain
- Title must be non-empty string
- Text must contain at least 50 characters

### 2. TextChunk

A portion of document text sized for embedding.

| Field | Type | Description |
|-------|------|-------------|
| text | string | Chunk content (~1500 chars) |
| chunk_index | integer | Position in parent document (0-indexed) |
| source_url | string | Parent document URL |
| source_title | string | Parent document title |

**Validation Rules**:
- Text must be non-empty and non-whitespace
- chunk_index must be >= 0
- source_url must match a processed document

### 3. Embedding

Vector representation stored in Qdrant.

| Field | Type | Description |
|-------|------|-------------|
| id | uuid | Unique identifier (generated) |
| vector | float[1024] | Cohere embedding vector |
| payload | object | Metadata (see below) |

**Payload Schema**:
```json
{
  "url": "https://ai-book-mu.vercel.app/docs/intro",
  "title": "Introduction",
  "chunk_index": 0,
  "text": "This is the chunk content..."
}
```

**Validation Rules**:
- Vector must be exactly 1024 dimensions
- All payload fields must be present
- Text in payload should match source chunk

## Qdrant Collection Schema

**Collection Name**: `rag_embedding`

```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload_schema": {
    "url": { "data_type": "keyword", "indexed": true },
    "title": { "data_type": "keyword", "indexed": true },
    "chunk_index": { "data_type": "integer", "indexed": false },
    "text": { "data_type": "text", "indexed": false }
  }
}
```

## Data Flow

```
┌─────────────┐     ┌──────────────┐     ┌────────────┐     ┌──────────┐
│ Docusaurus  │────▶│   Document   │────▶│ TextChunk  │────▶│ Embedding│
│    URL      │     │   (parsed)   │     │  (split)   │     │ (Qdrant) │
└─────────────┘     └──────────────┘     └────────────┘     └──────────┘
     │                     │                   │                  │
     │ get_all_urls()      │ extract_text()    │ chunk_text()     │ embed() +
     │                     │                   │                  │ save_to_qdrant()
     ▼                     ▼                   ▼                  ▼
  [URL list]          [title, text]       [chunk list]      [vectors stored]
```

## ID Generation Strategy

For Qdrant point IDs, use deterministic UUIDs based on URL + chunk_index:

```python
import uuid

def generate_point_id(url: str, chunk_index: int) -> str:
    """Generate deterministic UUID for upsert behavior."""
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"{url}#{chunk_index}"))
```

This ensures:
- Same URL + chunk_index always produces same ID
- Re-indexing updates existing vectors (upsert behavior)
- No duplicate vectors for same content
