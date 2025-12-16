# Quickstart: RAG Retrieval-Enabled Agent

**Feature**: 007-rag-retrieval-agent
**Time to Complete**: ~5 minutes

## Prerequisites

1. **Python 3.11+** with virtual environment activated
2. **Qdrant** collection already populated (via Feature 005)
3. **Environment variables** configured in `.env`

## Step 1: Install Dependencies

```bash
cd backend
pip install openai
```

The `openai` package is the only new dependency. Existing packages (`cohere`, `qdrant-client`, `python-dotenv`) are already installed.

## Step 2: Add OpenAI API Key

Add to your `backend/.env` file:

```env
# Existing keys (keep these)
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://your-cluster.qdrant.cloud
QDRANT_API_KEY=your-qdrant-key

# New key for agent
OPENAI_API_KEY=sk-your-openai-api-key
```

Get your OpenAI API key from: https://platform.openai.com/api-keys

## Step 3: Verify Prerequisites

Run the health check to ensure Qdrant is accessible:

```bash
python retrieve.py --health
```

Expected output:
```
Overall Status: HEALTHY
```

## Step 4: Run the Agent

### Interactive Mode

```bash
python agent.py
```

Then ask questions:
```
You: What is Isaac Sim?
You: How do I install ROS 2?
You: exit
```

### Single Query Mode

```bash
python agent.py "What is ROS 2?"
```

## Common Issues

### "Missing OPENAI_API_KEY"
Add your OpenAI API key to the `.env` file.

### "No relevant information found"
The question may not be covered in the embedded documentation. Try rephrasing or asking about topics in the Physical AI & Robotics book.

### Connection Errors
Run `python retrieve.py --health` to diagnose Qdrant connectivity issues.

## What's Next?

- Feature 008: REST API wrapper (FastAPI) for the agent
- Feature 009: Web UI for chat interface
