# Quickstart: RAG Chatbot Frontend Integration

**Feature**: 008-rag-chatbot-frontend
**Time to Complete**: ~15 minutes

## Prerequisites

1. **Feature 007 completed** - RAG agent (agent.py) working
2. **Python 3.11+** with backend virtual environment
3. **Node.js 18+** for Docusaurus frontend
4. **Environment variables** configured in `backend/.env`

## Step 1: Install Backend Dependencies

```bash
cd backend
.venv/Scripts/pip.exe install fastapi uvicorn
```

## Step 2: Start the Backend API

```bash
cd backend
.venv/Scripts/python.exe api.py
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete.
```

Verify with:
```bash
curl http://localhost:8000/health
```

## Step 3: Start the Frontend

In a new terminal:

```bash
cd book
npm run start
```

Expected output:
```
[SUCCESS] Docusaurus website is running at http://localhost:3000
```

## Step 4: Test the Chat Widget

1. Open http://localhost:3000 in your browser
2. Click the chat icon in the bottom-right corner
3. Type a question: "What is ROS 2?"
4. Verify you receive an answer with source citations

## Usage

### Basic Query

Click the chat widget, type your question, and press Enter or click Send.

### Context-Aware Query

1. Select text on any documentation page
2. Click "Ask about this" in the popup
3. The selected text becomes context for your question

### Clear History

Type "clear" in the chat or refresh the page.

## Common Issues

### "Failed to fetch" Error

Backend is not running. Start it with:
```bash
cd backend && .venv/Scripts/python.exe api.py
```

### CORS Error

Check that the backend allows the frontend origin in CORS config.

### Slow Responses

Normal - the RAG agent needs to:
1. Generate embedding
2. Search Qdrant
3. Call OpenAI for answer generation

First request may take 5-10 seconds.

## API Testing

### Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Isaac Sim?"}'
```

### Test with Context

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this further",
    "context": "Isaac Sim uses RTX ray tracing"
  }'
```

## Architecture

```
┌─────────────────┐     HTTP/JSON      ┌─────────────────┐
│   Docusaurus    │ ◄─────────────────► │    FastAPI      │
│   (React)       │   POST /chat        │    (Python)     │
│   localhost:3000│                     │   localhost:8000│
└─────────────────┘                     └─────────────────┘
                                               │
                                               ▼
                                        ┌─────────────────┐
                                        │   RAG Agent     │
                                        │   (agent.py)    │
                                        └─────────────────┘
                                               │
                               ┌───────────────┼───────────────┐
                               ▼               ▼               ▼
                        ┌──────────┐    ┌──────────┐    ┌──────────┐
                        │  Qdrant  │    │  Cohere  │    │  OpenAI  │
                        │ (vectors)│    │(embeddings)│   │  (LLM)   │
                        └──────────┘    └──────────┘    └──────────┘
```

## What's Next?

- Feature 009: Production deployment (Vercel/Railway)
- Feature 010: Add analytics/logging
- Feature 011: Authentication (optional)
