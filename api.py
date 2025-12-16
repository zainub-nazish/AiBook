"""
FastAPI REST API for RAG Chatbot

This module provides HTTP endpoints for the RAG chatbot, enabling
frontend integration with the Docusaurus documentation site.

Usage:
    python api.py                    # Start server on port 8000
    python api.py --port 8080        # Custom port
"""

import logging
import os
import time
import uuid
from typing import Optional

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

# Import from agent.py
from agent import (
    process_query,
    validate_env,
    SYSTEM_PROMPT,
    MAX_QUERY_LENGTH,
    DEFAULT_K,
    DEFAULT_THRESHOLD,
)
from retrieve import get_qdrant_client, get_cohere_client

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for querying Physical AI & Robotics documentation",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://ai-book-mu.vercel.app",
        "http://127.0.0.1:3000",
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Constants
MAX_CONTEXT_LENGTH = 2000


# Pydantic models
class QueryRequest(BaseModel):
    """Request model for chat endpoint."""
    question: str = Field(
        ...,
        min_length=1,
        max_length=MAX_QUERY_LENGTH,
        description="User's question text"
    )
    context: Optional[str] = Field(
        None,
        max_length=MAX_CONTEXT_LENGTH,
        description="Optional selected text from page"
    )
    session_id: Optional[str] = Field(
        None,
        description="Session identifier for conversation tracking"
    )
    k: int = Field(
        default=DEFAULT_K,
        ge=1,
        le=20,
        description="Number of chunks to retrieve"
    )


class Source(BaseModel):
    """Source citation in response."""
    title: str
    url: str
    score: float = Field(ge=0.0, le=1.0)


class QueryResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    sources: list[Source]
    session_id: str
    processing_time: Optional[float] = None


class ErrorResponse(BaseModel):
    """Error response model."""
    error: str
    message: str
    details: Optional[str] = None


class HealthResponse(BaseModel):
    """Health check response model."""
    status: str
    qdrant: str
    cohere: str
    openai: str
    error: Optional[str] = None


# Initialize clients (lazy loading)
_openai_client = None
_cohere_client = None
_qdrant_client = None


def get_openai_client():
    """Get or create OpenAI client."""
    global _openai_client
    if _openai_client is None:
        from openai import OpenAI
        _openai_client = OpenAI()
    return _openai_client


def get_clients():
    """Get all required API clients."""
    global _cohere_client, _qdrant_client

    if _cohere_client is None:
        _cohere_client = get_cohere_client()
    if _qdrant_client is None:
        _qdrant_client = get_qdrant_client()

    return get_openai_client(), _cohere_client, _qdrant_client


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check if API and dependencies are available."""
    qdrant_status = "disconnected"
    cohere_status = "unavailable"
    openai_status = "unavailable"
    error_msg = None

    try:
        # Check environment variables
        is_valid, missing = validate_env()
        if not is_valid:
            return HealthResponse(
                status="unhealthy",
                qdrant="unchecked",
                cohere="unchecked",
                openai="unchecked",
                error=f"Missing env vars: {', '.join(missing)}"
            )

        # Check Qdrant
        try:
            qdrant_client = get_qdrant_client()
            if qdrant_client:
                qdrant_status = "connected"
        except Exception as e:
            error_msg = f"Qdrant: {str(e)}"

        # Check Cohere
        try:
            cohere_client = get_cohere_client()
            if cohere_client:
                cohere_status = "available"
        except Exception as e:
            error_msg = error_msg or f"Cohere: {str(e)}"

        # Check OpenAI
        try:
            if os.getenv("OPENAI_API_KEY"):
                openai_status = "available"
        except Exception as e:
            error_msg = error_msg or f"OpenAI: {str(e)}"

        # Determine overall status
        if qdrant_status == "connected" and cohere_status == "available" and openai_status == "available":
            return HealthResponse(
                status="healthy",
                qdrant=qdrant_status,
                cohere=cohere_status,
                openai=openai_status
            )
        else:
            return HealthResponse(
                status="unhealthy",
                qdrant=qdrant_status,
                cohere=cohere_status,
                openai=openai_status,
                error=error_msg
            )

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="unhealthy",
            qdrant=qdrant_status,
            cohere=cohere_status,
            openai=openai_status,
            error=str(e)
        )


@app.post("/chat", response_model=QueryResponse)
async def chat(request: QueryRequest):
    """Process a chat query and return AI-generated answer with sources."""
    start_time = time.time()

    # Validate question
    question = request.question.strip()
    if not question:
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error="EMPTY_QUESTION",
                message="Question text cannot be empty"
            ).model_dump()
        )

    if len(question) > MAX_QUERY_LENGTH:
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error="QUESTION_TOO_LONG",
                message=f"Question exceeds maximum length of {MAX_QUERY_LENGTH} characters"
            ).model_dump()
        )

    # Validate context if provided
    context = request.context.strip() if request.context else None
    if context and len(context) > MAX_CONTEXT_LENGTH:
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error="CONTEXT_TOO_LONG",
                message=f"Context exceeds maximum length of {MAX_CONTEXT_LENGTH} characters"
            ).model_dump()
        )

    # Generate or use provided session ID
    session_id = request.session_id or str(uuid.uuid4())

    try:
        # Get clients
        openai_client, cohere_client, qdrant_client = get_clients()

        if not cohere_client:
            raise HTTPException(
                status_code=503,
                detail=ErrorResponse(
                    error="RETRIEVAL_ERROR",
                    message="Cohere client unavailable"
                ).model_dump()
            )

        if not qdrant_client:
            raise HTTPException(
                status_code=503,
                detail=ErrorResponse(
                    error="RETRIEVAL_ERROR",
                    message="Qdrant client unavailable"
                ).model_dump()
            )

        # Build query with context if provided
        full_query = question
        if context:
            full_query = f"Context from the page: \"{context}\"\n\nQuestion: {question}"

        # Create message history with system prompt
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]

        # Process query using agent
        answer, retrieval_results = process_query(
            full_query,
            messages,
            openai_client,
            cohere_client,
            qdrant_client,
            k=request.k,
            threshold=DEFAULT_THRESHOLD,
            debug=False,
            show_citations=False  # We'll format sources separately
        )

        # Format sources for response
        sources = []
        for result in retrieval_results:
            sources.append(Source(
                title=result.get('title', 'Unknown'),
                url=result.get('url', ''),
                score=result.get('score', 0.0)
            ))

        processing_time = time.time() - start_time

        return QueryResponse(
            answer=answer,
            sources=sources,
            session_id=session_id,
            processing_time=round(processing_time, 2)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error="AGENT_ERROR",
                message="Failed to generate response",
                details=str(e)
            ).model_dump()
        )


if __name__ == "__main__":
    import argparse
    import uvicorn

    parser = argparse.ArgumentParser(description="Start RAG Chatbot API server")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload")

    args = parser.parse_args()

    print(f"Starting RAG Chatbot API on http://{args.host}:{args.port}")
    print("API docs available at http://localhost:8000/docs")

    uvicorn.run(
        "api:app",
        host=args.host,
        port=args.port,
        reload=args.reload
    )
