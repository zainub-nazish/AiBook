"""
Embedding Pipeline for Docusaurus Documentation

This module crawls a Docusaurus site, extracts text content,
generates embeddings using Cohere, and stores them in Qdrant
for RAG-based retrieval.
"""

import logging
import os
import time
import uuid
import xml.etree.ElementTree as ET

import cohere
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()


def validate_credentials() -> bool:
    """Verify that all required API keys and URLs are present in environment."""
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'BASE_URL']
    missing = [var for var in required_vars if not os.getenv(var)]

    if missing:
        logger.error(f"Missing required environment variables: {', '.join(missing)}")
        return False

    logger.info("All credentials validated successfully")
    return True


def generate_point_id(url: str, chunk_index: int) -> str:
    """Generate deterministic UUID for Qdrant point based on URL and chunk index.

    This ensures:
    - Same URL + chunk_index always produces same ID
    - Re-indexing updates existing vectors (upsert behavior)
    - No duplicate vectors for same content
    """
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"{url}#{chunk_index}"))


def get_all_urls(base_url: str) -> list[str]:
    """Discover all documentation page URLs from sitemap.xml.

    Args:
        base_url: The base URL of the Docusaurus site

    Returns:
        List of documentation page URLs found in the sitemap
    """
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()

        # Parse XML sitemap
        root = ET.fromstring(response.content)

        # Handle XML namespace
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = []

        for url_elem in root.findall('.//ns:url/ns:loc', namespace):
            if url_elem.text:
                urls.append(url_elem.text)

        # If no namespace, try without it
        if not urls:
            for url_elem in root.findall('.//loc'):
                if url_elem.text:
                    urls.append(url_elem.text)

        # Fix URLs if sitemap has wrong domain (common with misconfigured Docusaurus)
        # Replace any incorrect domain with the actual base_url
        fixed_urls = []
        base_domain = base_url.rstrip('/')
        for url in urls:
            # Check if URL has a different domain than expected
            if not url.startswith(base_domain):
                # Extract path from the URL and prepend correct base
                from urllib.parse import urlparse
                parsed = urlparse(url)
                path = parsed.path
                fixed_url = f"{base_domain}{path}"
                fixed_urls.append(fixed_url)
                logger.debug(f"Fixed URL: {url} -> {fixed_url}")
            else:
                fixed_urls.append(url)

        logger.info(f"Discovered {len(fixed_urls)} URLs from sitemap")
        return fixed_urls

    except requests.RequestException as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        return []
    except ET.ParseError as e:
        logger.error(f"Failed to parse sitemap XML: {e}")
        return []


def extract_text_from_url(url: str) -> tuple[str, str] | None:
    """Fetch URL and extract clean text content.

    Args:
        url: The URL to fetch and extract text from

    Returns:
        Tuple of (title, text) or None if extraction fails
    """
    try:
        response = requests.get(url, timeout=30)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract title from <title> tag or first <h1>
        title = ""
        title_tag = soup.find('title')
        if title_tag:
            title = title_tag.get_text(strip=True)
        else:
            h1_tag = soup.find('h1')
            if h1_tag:
                title = h1_tag.get_text(strip=True)

        # Remove unwanted elements (nav, footer, header, scripts, styles)
        for element in soup.find_all(['nav', 'footer', 'header', 'script', 'style', 'aside']):
            element.decompose()

        # Find main content area (Docusaurus uses article or main)
        main_content = soup.find('article') or soup.find('main') or soup.find('body')

        if not main_content:
            logger.warning(f"No main content found for {url}")
            return None

        # Preserve code blocks by marking them
        for code_block in main_content.find_all(['pre', 'code']):
            # Keep code content as-is
            code_text = code_block.get_text()
            if code_block.name == 'pre':
                code_block.replace_with(f"\n```\n{code_text}\n```\n")
            else:
                code_block.replace_with(f"`{code_text}`")

        # Extract text
        text = main_content.get_text(separator='\n', strip=True)

        # Clean up excessive whitespace
        lines = [line.strip() for line in text.split('\n') if line.strip()]
        text = '\n'.join(lines)

        if len(text) < 50:
            logger.warning(f"Page has insufficient text content ({len(text)} chars): {url}")
            return None

        logger.info(f"Extracted {len(text)} chars from: {title or url}")
        return (title, text)

    except requests.RequestException as e:
        logger.error(f"Failed to fetch {url}: {e}")
        return None
    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return None


def chunk_text(text: str, chunk_size: int = 1500, overlap: int = 200) -> list[str]:
    """Split text into overlapping chunks for embedding.

    Args:
        text: The text to split
        chunk_size: Target size of each chunk in characters
        overlap: Number of characters to overlap between chunks

    Returns:
        List of text chunks
    """
    if not text or len(text) <= chunk_size:
        return [text] if text else []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # Try to break at a natural boundary (newline or period)
        if end < len(text):
            # Look for newline first
            newline_pos = text.rfind('\n', start + chunk_size // 2, end)
            if newline_pos > start:
                end = newline_pos + 1
            else:
                # Look for period
                period_pos = text.rfind('. ', start + chunk_size // 2, end)
                if period_pos > start:
                    end = period_pos + 2

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        # Move start position with overlap
        start = end - overlap if end < len(text) else len(text)

    logger.debug(f"Split text into {len(chunks)} chunks")
    return chunks


def embed(texts: list[str], api_key: str) -> list[list[float]]:
    """Generate embeddings using Cohere API with batching and retry logic.

    Args:
        texts: List of text chunks to embed
        api_key: Cohere API key

    Returns:
        List of embedding vectors (1024 dimensions each)
    """
    if not texts:
        return []

    # Filter out empty/whitespace texts
    valid_texts = []
    valid_indices = []
    for i, text in enumerate(texts):
        if text and text.strip():
            valid_texts.append(text)
            valid_indices.append(i)
        else:
            logger.warning(f"Skipping empty/whitespace text at index {i}")

    if not valid_texts:
        return []

    client = cohere.Client(api_key)
    all_embeddings = []
    batch_size = 96  # Cohere max batch size

    for batch_num, i in enumerate(range(0, len(valid_texts), batch_size)):
        batch = valid_texts[i:i + batch_size]
        max_retries = 3
        retry_delay = 1

        for attempt in range(max_retries):
            try:
                response = client.embed(
                    texts=batch,
                    model="embed-english-v3.0",
                    input_type="search_document",
                    embedding_types=["float"]
                )

                # Extract float embeddings
                batch_embeddings = response.embeddings.float_
                all_embeddings.extend(batch_embeddings)

                logger.info(f"Batch {batch_num + 1}: Generated {len(batch)} embeddings")
                break

            except cohere.errors.TooManyRequestsError:
                if attempt < max_retries - 1:
                    logger.warning(f"Rate limit hit, retrying in {retry_delay}s (attempt {attempt + 1}/{max_retries})")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                else:
                    logger.error(f"Failed batch {batch_num + 1} after {max_retries} retries due to rate limiting")
                    raise

            except Exception as e:
                logger.error(f"Error generating embeddings for batch {batch_num + 1}: {e}")
                raise

    logger.info(f"Generated total of {len(all_embeddings)} embeddings")
    return all_embeddings


def create_collection(client: QdrantClient, collection_name: str = "rag_embedding") -> bool:
    """Create Qdrant collection with correct vector configuration.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to create

    Returns:
        True if collection was created or already exists, False on error
    """
    try:
        # Check if collection already exists
        collections = client.get_collections().collections
        existing_names = [c.name for c in collections]

        if collection_name in existing_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create collection with correct vector config for Cohere embed-english-v3.0
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=models.Distance.COSINE
            )
        )

        logger.info(f"Created collection '{collection_name}' with 1024-dim vectors")
        return True

    except Exception as e:
        logger.error(f"Failed to create collection '{collection_name}': {e}")
        return False


def save_chunk_to_qdrant(
    client: QdrantClient,
    collection_name: str,
    vectors: list[list[float]],
    payloads: list[dict]
) -> bool:
    """Upsert vectors with metadata to Qdrant.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection
        vectors: List of embedding vectors
        payloads: List of metadata dicts with url, title, chunk_index, text

    Returns:
        True on success, False on error
    """
    if not vectors or not payloads:
        logger.warning("No vectors or payloads to save")
        return True

    if len(vectors) != len(payloads):
        logger.error(f"Vector count ({len(vectors)}) doesn't match payload count ({len(payloads)})")
        return False

    max_retries = 2
    retry_delay = 1

    for attempt in range(max_retries):
        try:
            # Generate deterministic point IDs
            points = []
            for i, (vector, payload) in enumerate(zip(vectors, payloads)):
                point_id = generate_point_id(payload['url'], payload['chunk_index'])
                points.append(
                    models.PointStruct(
                        id=point_id,
                        vector=vector,
                        payload=payload
                    )
                )

            # Upsert points (update if exists, insert if new)
            client.upsert(
                collection_name=collection_name,
                points=points
            )

            logger.info(f"Saved {len(points)} vectors to Qdrant collection '{collection_name}'")
            return True

        except Exception as e:
            if attempt < max_retries - 1:
                logger.warning(f"Qdrant upsert failed, retrying in {retry_delay}s: {e}")
                time.sleep(retry_delay)
                retry_delay *= 2
            else:
                logger.error(f"Failed to save vectors to Qdrant after {max_retries} attempts: {e}")
                return False

    return False


def main():
    """Orchestrate the complete embedding pipeline.

    Flow:
    1. Validate credentials
    2. Get all URLs from sitemap
    3. For each URL: extract text → chunk → embed → save to Qdrant
    4. Log summary
    """
    start_time = time.time()

    logger.info("=" * 60)
    logger.info("Starting Embedding Pipeline")
    logger.info("=" * 60)

    # Step 1: Validate credentials
    if not validate_credentials():
        logger.error("Pipeline aborted: Missing credentials")
        return

    # Get environment variables
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    base_url = os.getenv('BASE_URL')

    # Step 2: Initialize Qdrant client
    logger.info(f"Connecting to Qdrant at {qdrant_url}")
    try:
        qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        return

    # Step 3: Create collection
    collection_name = "rag_embedding"
    if not create_collection(qdrant_client, collection_name):
        logger.error("Pipeline aborted: Failed to create collection")
        return

    # Step 4: Get all URLs from sitemap
    urls = get_all_urls(base_url)
    if not urls:
        logger.error("Pipeline aborted: No URLs found in sitemap")
        return

    # Step 5: Process each URL
    total_pages = 0
    total_chunks = 0
    total_vectors = 0
    failed_pages = 0

    for i, url in enumerate(urls):
        logger.info(f"Processing page {i + 1}/{len(urls)}: {url}")

        # Extract text
        result = extract_text_from_url(url)
        if result is None:
            failed_pages += 1
            continue

        title, text = result
        total_pages += 1

        # Chunk text
        chunks = chunk_text(text)
        if not chunks:
            logger.warning(f"No chunks generated for {url}")
            continue

        chunk_count = len(chunks)
        total_chunks += chunk_count
        logger.info(f"Created {chunk_count} chunks from page")

        # Generate embeddings
        try:
            embeddings = embed(chunks, cohere_api_key)
        except Exception as e:
            logger.error(f"Failed to generate embeddings for {url}: {e}")
            continue

        # Prepare payloads
        payloads = [
            {
                'url': url,
                'title': title,
                'chunk_index': idx,
                'text': chunk
            }
            for idx, chunk in enumerate(chunks)
        ]

        # Save to Qdrant
        if save_chunk_to_qdrant(qdrant_client, collection_name, embeddings, payloads):
            total_vectors += len(embeddings)
        else:
            logger.error(f"Failed to save vectors for {url}")

    # Step 6: Log summary
    elapsed_time = time.time() - start_time
    logger.info("=" * 60)
    logger.info("Pipeline Complete!")
    logger.info("=" * 60)
    logger.info(f"Summary:")
    logger.info(f"  - Total URLs processed: {len(urls)}")
    logger.info(f"  - Successful pages: {total_pages}")
    logger.info(f"  - Failed pages: {failed_pages}")
    logger.info(f"  - Total chunks created: {total_chunks}")
    logger.info(f"  - Total vectors stored: {total_vectors}")
    logger.info(f"  - Elapsed time: {elapsed_time:.2f} seconds")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
