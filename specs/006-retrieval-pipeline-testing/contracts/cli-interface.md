# CLI Interface Contract: retrieve.py

**Feature**: 006-retrieval-pipeline-testing
**Date**: 2025-12-15

## Overview

`retrieve.py` is a command-line tool for testing the RAG retrieval pipeline. It provides similarity search and health check functionality.

## Commands

### 1. Similarity Search (Default)

**Usage**:
```bash
python retrieve.py "your search query here"
python retrieve.py "ROS 2 nodes" --k 10
python retrieve.py "robot navigation" --k 5 --threshold 0.5
```

**Arguments**:

| Argument | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| query | string | Yes | - | The search text |
| --k | int | No | 5 | Number of results |
| --threshold | float | No | 0.0 | Minimum similarity score (0.0-1.0) |

**Output Format**:
```
Query: "ROS 2 nodes"
Found 5 results in 0.234s

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[1] Score: 92.3%
Title: Introduction to ROS 2 Nodes
URL: https://ai-book-mu.vercel.app/docs/module-01-ros2/chapter-02
─────────────────────────────────────────────────────────────
ROS 2 nodes are the fundamental building blocks of any ROS 2
application. A node is an executable that uses ROS 2 to communicate
with other nodes...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[2] Score: 87.1%
...
```

**Exit Codes**:

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Configuration error (missing env vars) |
| 2 | Connection error (Qdrant unreachable) |
| 3 | Query error (invalid input) |

### 2. Health Check

**Usage**:
```bash
python retrieve.py --health
```

**Output Format**:
```
RAG Pipeline Health Check
═════════════════════════════════════════════════════════════

Component          Status    Details
─────────────────────────────────────────────────────────────
Qdrant Connection  ✓ OK      https://xxx.cloud.qdrant.io:6333
Collection         ✓ OK      rag_embedding
Vector Count       ✓ OK      1,234 vectors
Sample Query       ✓ OK      5 results in 0.156s

═════════════════════════════════════════════════════════════
Overall Status: HEALTHY
```

**Error Output**:
```
RAG Pipeline Health Check
═════════════════════════════════════════════════════════════

Component          Status    Details
─────────────────────────────────────────────────────────────
Qdrant Connection  ✗ FAIL    Connection refused
Collection         - SKIP    (requires connection)
Vector Count       - SKIP    (requires connection)
Sample Query       - SKIP    (requires connection)

═════════════════════════════════════════════════════════════
Overall Status: UNHEALTHY

Errors:
  - Failed to connect to Qdrant: Connection refused
  - Check QDRANT_URL and QDRANT_API_KEY in .env
```

## Environment Variables

Required (same as main.py):

| Variable | Description | Example |
|----------|-------------|---------|
| COHERE_API_KEY | Cohere API key for embeddings | `xxx...` |
| QDRANT_URL | Qdrant cloud instance URL | `https://xxx.cloud.qdrant.io:6333` |
| QDRANT_API_KEY | Qdrant API key | `eyJ...` |

## Error Messages

| Scenario | Error Message |
|----------|---------------|
| Missing COHERE_API_KEY | `Error: COHERE_API_KEY not found in environment. Check .env file.` |
| Missing QDRANT_URL | `Error: QDRANT_URL not found in environment. Check .env file.` |
| Qdrant unreachable | `Error: Cannot connect to Qdrant at {url}. Check network and credentials.` |
| Collection not found | `Error: Collection 'rag_embedding' not found. Run main.py first to index documents.` |
| Empty query | `Error: Query text cannot be empty.` |
| No results | `No results found for query "{query}". Try a different search term.` |
