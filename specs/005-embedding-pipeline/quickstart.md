# Quickstart: Embedding Pipeline

**Feature**: 005-embedding-pipeline
**Date**: 2025-12-15

## Prerequisites

1. **Python 3.11+** installed
2. **UV** package manager installed ([install guide](https://docs.astral.sh/uv/getting-started/installation/))
3. **Cohere API key** ([get one here](https://dashboard.cohere.com/api-keys))
4. **Qdrant instance** ([free cloud tier](https://cloud.qdrant.io/))

## Setup

### 1. Create Backend Folder

```bash
cd book1
mkdir backend
cd backend
```

### 2. Initialize UV Project

```bash
uv init
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv
```

### 3. Configure Environment

Create `.env` file:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
BASE_URL=https://ai-book-mu.vercel.app
```

### 4. Run Pipeline

```bash
uv run main.py
```

## Expected Output

```
2025-12-15 10:00:00 - INFO - Starting pipeline for https://ai-book-mu.vercel.app
2025-12-15 10:00:01 - INFO - Discovered 25 URLs from sitemap
2025-12-15 10:00:02 - INFO - Processing: Introduction
2025-12-15 10:00:03 - INFO - Created 3 chunks from page
...
2025-12-15 10:05:00 - INFO - Pipeline complete!
2025-12-15 10:05:00 - INFO - Summary: 25 pages, 150 chunks, 150 vectors stored
```

## Verify in Qdrant

After running, verify vectors in Qdrant Cloud dashboard:
1. Go to your Qdrant cluster
2. Find collection `rag_embedding`
3. Check points count matches chunks created

## Troubleshooting

### "Invalid API key" error
- Verify COHERE_API_KEY is correct
- Check key has embed permissions

### "Connection refused" to Qdrant
- Verify QDRANT_URL is correct
- Check QDRANT_API_KEY is valid
- Ensure cluster is running

### "Rate limit exceeded"
- Pipeline will auto-retry with backoff
- If persistent, wait a few minutes

### "No URLs found"
- Check BASE_URL is accessible
- Verify sitemap exists at /sitemap.xml

## File Structure After Setup

```
backend/
├── main.py              # Pipeline script
├── .env                  # Your API keys (gitignored)
├── .env.example          # Template for others
├── pyproject.toml        # UV project config
└── .python-version       # Python version lock
```
