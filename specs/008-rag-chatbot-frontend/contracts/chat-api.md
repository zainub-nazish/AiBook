# Chat API Contract

**Feature**: 008-rag-chatbot-frontend
**Date**: 2025-12-16
**Base URL**: `http://localhost:8000`

## Endpoints

### POST /chat

Send a question to the RAG agent and receive an AI-generated answer.

#### Request

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "question": "What is ROS 2?",
  "context": "I'm reading about robot perception...",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "k": 5
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| question | string | Yes | User's question (1-8000 chars) |
| context | string | No | Selected page text (max 2000 chars) |
| session_id | string | No | Session ID for conversation tracking |
| k | integer | No | Number of chunks to retrieve (1-20, default 5) |

#### Response - Success (200 OK)

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is an open-source middleware framework for building robot applications...",
  "sources": [
    {
      "title": "Introduction to ROS 2",
      "url": "https://ai-book-mu.vercel.app/docs/module-01/chapter-02-ros2",
      "score": 0.89
    },
    {
      "title": "ROS 2 Architecture",
      "url": "https://ai-book-mu.vercel.app/docs/module-01/chapter-03-architecture",
      "score": 0.76
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "processing_time": 3.45
}
```

| Field | Type | Description |
|-------|------|-------------|
| answer | string | AI-generated response text |
| sources | array | List of source citations |
| sources[].title | string | Document/section title |
| sources[].url | string | Link to documentation |
| sources[].score | number | Relevance score (0.0-1.0) |
| session_id | string | Session identifier |
| processing_time | number | Time taken in seconds |

#### Response - Error (4xx/5xx)

```json
{
  "error": "EMPTY_QUESTION",
  "message": "Question text cannot be empty",
  "details": null
}
```

| Field | Type | Description |
|-------|------|-------------|
| error | string | Error code identifier |
| message | string | Human-readable error message |
| details | string | Additional context (optional) |

**Error Codes**:

| Code | HTTP Status | Description |
|------|-------------|-------------|
| EMPTY_QUESTION | 400 | Question is empty or whitespace |
| QUESTION_TOO_LONG | 400 | Question exceeds 8000 characters |
| CONTEXT_TOO_LONG | 400 | Context exceeds 2000 characters |
| RETRIEVAL_ERROR | 500 | Failed to search documentation |
| AGENT_ERROR | 500 | Failed to generate response |
| TIMEOUT | 504 | Request exceeded time limit |

---

### GET /health

Check if the API is running and dependencies are accessible.

#### Response - Success (200 OK)

```json
{
  "status": "healthy",
  "qdrant": "connected",
  "cohere": "available",
  "openai": "available"
}
```

#### Response - Unhealthy (503)

```json
{
  "status": "unhealthy",
  "qdrant": "disconnected",
  "cohere": "available",
  "openai": "available",
  "error": "Cannot connect to Qdrant"
}
```

---

## CORS Configuration

The API must allow requests from the Docusaurus frontend:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://ai-book-mu.vercel.app"],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

---

## Examples

### Basic Question

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Isaac Sim?"}'
```

### Question with Context

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this in more detail",
    "context": "Isaac Sim uses RTX ray tracing for photorealistic rendering."
  }'
```

### Health Check

```bash
curl http://localhost:8000/health
```

---

## OpenAPI Schema

```yaml
openapi: 3.0.0
info:
  title: RAG Chatbot API
  version: 1.0.0
  description: API for querying Physical AI & Robotics documentation

paths:
  /chat:
    post:
      summary: Send question to RAG agent
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/QueryRequest'
      responses:
        '200':
          description: Successful response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/QueryResponse'
        '400':
          description: Bad request
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /health:
    get:
      summary: Health check
      responses:
        '200':
          description: API is healthy
        '503':
          description: API is unhealthy

components:
  schemas:
    QueryRequest:
      type: object
      required:
        - question
      properties:
        question:
          type: string
          minLength: 1
          maxLength: 8000
        context:
          type: string
          maxLength: 2000
        session_id:
          type: string
          format: uuid
        k:
          type: integer
          minimum: 1
          maximum: 20
          default: 5

    QueryResponse:
      type: object
      required:
        - answer
        - sources
        - session_id
      properties:
        answer:
          type: string
        sources:
          type: array
          items:
            $ref: '#/components/schemas/Source'
        session_id:
          type: string
        processing_time:
          type: number

    Source:
      type: object
      required:
        - title
        - url
        - score
      properties:
        title:
          type: string
        url:
          type: string
          format: uri
        score:
          type: number
          minimum: 0
          maximum: 1

    ErrorResponse:
      type: object
      required:
        - error
        - message
      properties:
        error:
          type: string
        message:
          type: string
        details:
          type: string
```
