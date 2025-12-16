# Quickstart: Retrieval Pipeline Testing

**Feature**: 006-retrieval-pipeline-testing
**Date**: 2025-12-15

## Prerequisites

1. **Python 3.11+** installed
2. **Embedding pipeline run** - vectors must be stored in Qdrant (run `main.py` first)
3. **Environment configured** - `.env` file with API keys

## Setup

```bash
# Navigate to backend folder
cd backend

# Activate virtual environment (if not active)
# Windows:
.venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# Verify .env exists with required variables
cat .env
# Should show: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
```

## Usage

### Run a Similarity Query

```bash
# Basic query (returns top 5 results)
python retrieve.py "ROS 2 nodes"

# Specify number of results
python retrieve.py "robot navigation" --k 10

# Filter by minimum similarity score
python retrieve.py "Isaac Sim" --k 5 --threshold 0.7
```

### Run Health Check

```bash
# Verify all pipeline components
python retrieve.py --health
```

## Expected Output

### Successful Query
```
Query: "ROS 2 nodes"
Found 5 results in 0.234s

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[1] Score: 92.3%
Title: Introduction to ROS 2 Nodes
URL: https://ai-book-mu.vercel.app/docs/module-01-ros2/...
─────────────────────────────────────────────
ROS 2 nodes are the fundamental building blocks...
```

### Successful Health Check
```
RAG Pipeline Health Check
═══════════════════════════════════════════════

Component          Status    Details
───────────────────────────────────────────────
Qdrant Connection  ✓ OK      Connected
Collection         ✓ OK      rag_embedding
Vector Count       ✓ OK      1,234 vectors
Sample Query       ✓ OK      5 results in 0.156s

═══════════════════════════════════════════════
Overall Status: HEALTHY
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Collection not found" | Run `python main.py` first to index documents |
| "Connection refused" | Check QDRANT_URL in .env, verify network connectivity |
| "Rate limit" | Wait and retry, or check Cohere API quota |
| No results | Try different search terms, check if collection has vectors |

## Test Queries

Try these queries to validate different documentation modules:

| Query | Expected Module |
|-------|-----------------|
| "ROS 2 nodes" | Module 01 - ROS2 Fundamentals |
| "Gazebo simulation" | Module 02 - Digital Twin |
| "Isaac Sim" | Module 03 - NVIDIA Isaac |
| "vision language action" | Module 04 - VLA |
