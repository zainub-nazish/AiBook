# API Contracts: Embedding Pipeline

**Feature**: 005-embedding-pipeline
**Date**: 2025-12-15

## Overview

This document defines the external API contracts for the embedding pipeline. Since this is a CLI tool (not a web service), contracts focus on:
1. Cohere API usage
2. Qdrant API usage
3. CLI interface

## 1. Cohere Embed API Contract

### Request

```http
POST https://api.cohere.ai/v1/embed
Authorization: Bearer {COHERE_API_KEY}
Content-Type: application/json
```

```json
{
  "model": "embed-english-v3.0",
  "texts": ["text1", "text2", "...up to 96 texts"],
  "input_type": "search_document",
  "embedding_types": ["float"]
}
```

### Response (Success)

```json
{
  "id": "request-id",
  "texts": ["text1", "text2"],
  "embeddings": {
    "float": [
      [0.123, -0.456, ...],  // 1024 floats
      [0.789, -0.012, ...]   // 1024 floats
    ]
  },
  "meta": {
    "api_version": {"version": "1"},
    "billed_units": {"input_tokens": 100}
  }
}
```

### Response (Error - Rate Limit)

```json
{
  "message": "rate limit exceeded"
}
```
HTTP Status: 429

### Error Handling

| Status | Action |
|--------|--------|
| 200 | Extract embeddings from response |
| 429 | Exponential backoff: 1s, 2s, 4s, then fail |
| 401 | Fail with "Invalid API key" message |
| 5xx | Retry once, then fail |

## 2. Qdrant API Contracts

### Create Collection

```http
PUT {QDRANT_URL}/collections/rag_embedding
Authorization: Api-Key {QDRANT_API_KEY}
Content-Type: application/json
```

```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  }
}
```

### Upsert Points

```http
PUT {QDRANT_URL}/collections/rag_embedding/points
Authorization: Api-Key {QDRANT_API_KEY}
Content-Type: application/json
```

```json
{
  "points": [
    {
      "id": "uuid-string",
      "vector": [0.123, -0.456, ...],
      "payload": {
        "url": "https://example.com/page",
        "title": "Page Title",
        "chunk_index": 0,
        "text": "Chunk content..."
      }
    }
  ]
}
```

### Response (Success)

```json
{
  "result": {
    "operation_id": 123,
    "status": "completed"
  },
  "status": "ok",
  "time": 0.001
}
```

## 3. CLI Interface Contract

### Command

```bash
uv run main.py
```

### Environment Variables (Required)

| Variable | Description | Example |
|----------|-------------|---------|
| COHERE_API_KEY | Cohere API key | co-xxxx |
| QDRANT_URL | Qdrant instance URL | https://xxx.qdrant.io |
| QDRANT_API_KEY | Qdrant API key | qdrant-xxxx |
| BASE_URL | Docusaurus site URL | https://ai-book-mu.vercel.app |

### Output (stdout)

```text
2025-12-15 10:00:00 - INFO - Starting pipeline for https://ai-book-mu.vercel.app
2025-12-15 10:00:01 - INFO - Discovered 25 URLs from sitemap
2025-12-15 10:00:02 - INFO - Processing: /docs/intro
2025-12-15 10:00:03 - INFO - Extracted 1500 chars, created 2 chunks
2025-12-15 10:00:04 - INFO - Generated embeddings for 2 chunks
2025-12-15 10:00:05 - INFO - Saved 2 vectors to Qdrant
...
2025-12-15 10:05:00 - INFO - Pipeline complete: 25 pages, 150 chunks, 150 vectors
```

### Output (stderr - errors)

```text
2025-12-15 10:00:00 - ERROR - Failed to fetch /docs/broken: 404 Not Found
2025-12-15 10:00:00 - WARNING - Skipping empty page: /docs/empty
```

### Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Missing environment variables |
| 2 | Cohere API error (after retries) |
| 3 | Qdrant connection error |
| 4 | No pages found to process |

## 4. Data Payload Contract

### Qdrant Point Payload

All stored vectors must include this payload structure:

```json
{
  "url": {
    "type": "string",
    "required": true,
    "description": "Full URL of source page",
    "example": "https://ai-book-mu.vercel.app/docs/intro"
  },
  "title": {
    "type": "string",
    "required": true,
    "description": "Page title",
    "example": "Introduction"
  },
  "chunk_index": {
    "type": "integer",
    "required": true,
    "description": "0-indexed chunk position",
    "example": 0
  },
  "text": {
    "type": "string",
    "required": true,
    "description": "Chunk text content",
    "example": "This documentation covers..."
  }
}
```
