# Research: Embedding Pipeline Setup

**Feature**: 005-embedding-pipeline
**Date**: 2025-12-15

## Technology Decisions

### 1. Package Manager: UV

**Decision**: Use UV for Python package management

**Rationale**:
- Extremely fast package resolution and installation (10-100x faster than pip)
- Built-in virtual environment management
- Compatible with pyproject.toml standard
- Modern replacement for pip + virtualenv

**Alternatives Considered**:
- pip + venv: Slower, requires separate tools
- Poetry: More complex, slower than UV
- Conda: Overkill for this use case

### 2. Web Scraping: BeautifulSoup4 + Requests

**Decision**: Use requests for HTTP and BeautifulSoup4 for HTML parsing

**Rationale**:
- Simple, well-documented libraries
- BeautifulSoup handles malformed HTML gracefully
- Sufficient for static Docusaurus pages
- Low memory footprint

**Alternatives Considered**:
- Scrapy: Overkill for single-site crawling
- Playwright/Selenium: Unnecessary for static content
- httpx: Good but requests is simpler

### 3. Cohere Embedding Model

**Decision**: Use `embed-english-v3.0` model with `search_document` input type

**Rationale**:
- 1024-dimensional embeddings (good balance of quality/storage)
- Optimized for semantic search use cases
- `search_document` type for indexing, `search_query` for retrieval
- Up to 96 texts per API call for batching

**Key Parameters**:
- `model`: "embed-english-v3.0"
- `input_type`: "search_document"
- `embedding_types`: ["float"]

**Rate Limits**:
- Free tier: 100 API calls/minute
- Implement exponential backoff with max 3 retries

### 4. Qdrant Vector Database

**Decision**: Use Qdrant Cloud with qdrant-client Python SDK

**Rationale**:
- Free tier available for development
- Simple REST/gRPC API
- Native Python client
- Built-in HNSW indexing

**Collection Configuration**:
- Collection name: `rag_embedding`
- Vector size: 1024 (matches Cohere embed-english-v3.0)
- Distance metric: Cosine similarity

**Payload Schema**:
```json
{
  "url": "string",
  "title": "string",
  "chunk_index": "integer",
  "text": "string"
}
```

### 5. Text Chunking Strategy

**Decision**: Simple character-based chunking with overlap

**Rationale**:
- Cohere embed-english-v3.0 max tokens: 512
- Target chunk size: ~1500 characters (~400 tokens)
- Overlap: 200 characters for context continuity
- Simple to implement, effective for documentation

**Alternatives Considered**:
- Semantic chunking: More complex, marginal benefit
- Sentence-based: Can create very uneven chunks
- LangChain text splitters: Adds unnecessary dependency

### 6. URL Discovery Strategy

**Decision**: Parse Docusaurus sitemap.xml first, fallback to link crawling

**Rationale**:
- Docusaurus generates sitemap.xml at `/sitemap.xml`
- Sitemap provides complete URL list without crawling
- Fallback to parsing navigation links if sitemap unavailable

**Target Site**: https://ai-book-mu.vercel.app/
- Sitemap URL: https://ai-book-mu.vercel.app/sitemap.xml

## Environment Variables

```bash
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
BASE_URL=https://ai-book-mu.vercel.app
```

## Dependencies (pyproject.toml)

```toml
[project]
name = "embedding-pipeline"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "python-dotenv>=1.0.0",
]

[project.optional-dependencies]
dev = ["pytest>=8.0.0"]
```

## Function Signatures

Based on user requirements, the following functions will be implemented in `main.py`:

```python
def get_all_urls(base_url: str) -> list[str]:
    """Discover all documentation page URLs from sitemap or crawling."""

def extract_text_from_url(url: str) -> tuple[str, str]:
    """Fetch URL and extract clean text. Returns (title, text)."""

def chunk_text(text: str, chunk_size: int = 1500, overlap: int = 200) -> list[str]:
    """Split text into overlapping chunks for embedding."""

def embed(texts: list[str], api_key: str) -> list[list[float]]:
    """Generate embeddings using Cohere API. Handles batching."""

def create_collection(client: QdrantClient, collection_name: str = "rag_embedding"):
    """Create Qdrant collection with correct vector config."""

def save_chunk_to_qdrant(
    client: QdrantClient,
    collection_name: str,
    vectors: list[list[float]],
    payloads: list[dict]
):
    """Upsert vectors with metadata to Qdrant."""

def main():
    """Orchestrate the complete pipeline."""
```

## Error Handling Strategy

1. **Invalid URL**: Log error, skip page, continue with others
2. **Cohere Rate Limit**: Exponential backoff (1s, 2s, 4s), max 3 retries
3. **Qdrant Connection Error**: Retry once, then fail with clear message
4. **Empty Content**: Skip with warning logged
5. **Missing API Keys**: Fail fast with clear error message

## Logging Format

```python
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
```

Log events:
- Pipeline start with URL count
- Each page processed
- Embedding batch complete
- Qdrant upsert complete
- Any errors/warnings
- Pipeline complete with summary
