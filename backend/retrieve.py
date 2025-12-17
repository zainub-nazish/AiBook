"""
Retrieval Pipeline Testing for RAG Documentation Search

This module provides similarity search and health check functionality
to test the RAG retrieval pipeline. It connects to Qdrant, generates
query embeddings using Cohere, and retrieves relevant document chunks.

Usage:
    python retrieve.py "your search query"
    python retrieve.py "ROS 2 nodes" --k 10
    python retrieve.py --health
"""

import argparse
import logging
import os
import sys
import time

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Configure logging (same format as main.py)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Constants
COLLECTION_NAME = "rag_embedding"
DEFAULT_K = 5
DEFAULT_THRESHOLD = 0.0
MODEL = "embed-english-v3.0"
MAX_QUERY_LENGTH = 8000
MAX_TEXT_DISPLAY = 300


def validate_env() -> tuple[bool, list[str]]:
    """Validate required environment variables are present.

    Returns:
        Tuple of (is_valid, list of missing variable names)
    """
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing = [var for var in required_vars if not os.getenv(var)]
    return (len(missing) == 0, missing)


def get_qdrant_client() -> QdrantClient | None:
    """Initialize and return Qdrant client.

    Returns:
        QdrantClient instance or None if connection fails
    """
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    try:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        return None


def get_cohere_client() -> cohere.Client | None:
    """Initialize and return Cohere client.

    Returns:
        Cohere Client instance or None if initialization fails
    """
    api_key = os.getenv('COHERE_API_KEY')
    try:
        return cohere.Client(api_key)
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {e}")
        return None


def embed_query(text: str, cohere_client: cohere.Client) -> list[float] | None:
    """Generate embedding for query text using Cohere.

    Args:
        text: The query text to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of floats (1024 dimensions) or None on error
    """
    try:
        response = cohere_client.embed(
            texts=[text],
            model=MODEL,
            input_type="search_query",
            embedding_types=["float"]
        )
        return response.embeddings.float_[0]
    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        return None


def search_similar(
    qdrant_client: QdrantClient,
    query_vector: list[float],
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD
) -> list[dict]:
    """Execute similarity search in Qdrant.

    Args:
        qdrant_client: Initialized Qdrant client
        query_vector: Query embedding vector
        k: Number of results to return
        threshold: Minimum similarity score (0.0-1.0)

    Returns:
        List of result dicts with score, text, url, title, chunk_index
    """
    try:
        # Use query_points for newer qdrant-client versions
        results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=k,
            score_threshold=threshold if threshold > 0 else None
        )

        formatted_results = []
        for point in results.points:
            formatted_results.append({
                'score': point.score,
                'text': point.payload.get('text', ''),
                'url': point.payload.get('url', ''),
                'title': point.payload.get('title', ''),
                'chunk_index': point.payload.get('chunk_index', 0)
            })

        return formatted_results
    except Exception as e:
        logger.error(f"Search failed: {e}")
        return []


def sanitize_text(text: str) -> str:
    """Remove non-ASCII characters for Windows console compatibility.

    Args:
        text: Input text that may contain unicode

    Returns:
        ASCII-safe text
    """
    return text.encode('ascii', 'ignore').decode('ascii')


def format_result(result: dict, index: int) -> str:
    """Format a single search result for display.

    Args:
        result: Result dict with score, text, url, title
        index: Result number (1-based)

    Returns:
        Formatted string for display
    """
    score_percent = result['score'] * 100
    text = sanitize_text(result['text'])
    title = sanitize_text(result['title'])

    # Truncate long text
    if len(text) > MAX_TEXT_DISPLAY:
        text = text[:MAX_TEXT_DISPLAY] + "..."

    return f"""
[{index}] Score: {score_percent:.1f}%
Title: {title}
URL: {result['url']}
{'-' * 60}
{text}
"""


def display_results(results: list[dict], query: str, elapsed: float) -> None:
    """Display search results to console.

    Args:
        results: List of result dicts
        query: Original query string
        elapsed: Time taken in seconds
    """
    print(f'\nQuery: "{query}"')

    if not results:
        print(f"\nNo results found for query \"{query}\". Try a different search term.")
        return

    print(f"Found {len(results)} results in {elapsed:.3f}s")
    print("\n" + "=" * 60)

    for i, result in enumerate(results, 1):
        print(format_result(result, i))
        print("=" * 60)


def search(
    query: str,
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD
) -> int:
    """Main search function orchestrating the retrieval pipeline.

    Args:
        query: Search query text
        k: Number of results to return
        threshold: Minimum similarity score

    Returns:
        Exit code (0=success, 1=config error, 2=connection error, 3=query error)
    """
    start_time = time.time()

    # Validate input
    if not query or not query.strip():
        print("Error: Query text cannot be empty.")
        return 3

    query = query.strip()
    if len(query) > MAX_QUERY_LENGTH:
        print(f"Error: Query too long. Maximum {MAX_QUERY_LENGTH} characters allowed.")
        return 3

    # Validate environment
    is_valid, missing = validate_env()
    if not is_valid:
        print(f"Error: Missing environment variables: {', '.join(missing)}")
        print("Check your .env file.")
        return 1

    # Initialize clients
    cohere_client = get_cohere_client()
    if not cohere_client:
        print("Error: Failed to initialize Cohere client. Check COHERE_API_KEY.")
        return 1

    qdrant_client = get_qdrant_client()
    if not qdrant_client:
        print("Error: Cannot connect to Qdrant. Check QDRANT_URL and QDRANT_API_KEY.")
        return 2

    # Generate query embedding
    logger.info(f"Generating embedding for query: {query[:50]}...")
    query_vector = embed_query(query, cohere_client)
    if not query_vector:
        print("Error: Failed to generate query embedding.")
        return 3

    # Execute search
    logger.info(f"Searching for top {k} results...")
    results = search_similar(qdrant_client, query_vector, k, threshold)

    elapsed = time.time() - start_time

    # Display results
    display_results(results, query, elapsed)

    logger.info(f"Search completed in {elapsed:.3f}s")
    return 0


def check_qdrant_connection(client: QdrantClient) -> tuple[bool, str]:
    """Check if Qdrant is reachable.

    Args:
        client: Qdrant client instance

    Returns:
        Tuple of (success, status message)
    """
    try:
        # Try to get collections as a connectivity test
        client.get_collections()
        return (True, "Connected")
    except Exception as e:
        return (False, f"Connection failed: {e}")


def check_collection_exists(client: QdrantClient) -> tuple[bool, str, int]:
    """Check if the RAG collection exists and get vector count.

    Args:
        client: Qdrant client instance

    Returns:
        Tuple of (exists, collection name, vector count)
    """
    try:
        collection_info = client.get_collection(COLLECTION_NAME)
        vector_count = collection_info.points_count
        return (True, COLLECTION_NAME, vector_count)
    except Exception as e:
        return (False, str(e), 0)


def run_sample_query(
    qdrant_client: QdrantClient,
    cohere_client: cohere.Client
) -> tuple[bool, float, int]:
    """Run a sample query to verify end-to-end pipeline.

    Args:
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere client

    Returns:
        Tuple of (success, time_ms, result_count)
    """
    try:
        start = time.time()

        # Generate test embedding
        test_query = "robot"
        vector = embed_query(test_query, cohere_client)
        if not vector:
            return (False, 0.0, 0)

        # Execute search
        results = search_similar(qdrant_client, vector, k=5)

        elapsed_ms = (time.time() - start) * 1000
        return (True, elapsed_ms, len(results))
    except Exception as e:
        logger.error(f"Sample query failed: {e}")
        return (False, 0.0, 0)


def format_health_report(checks: dict) -> str:
    """Format health check results as a table.

    Args:
        checks: Dict with check results

    Returns:
        Formatted health report string
    """
    lines = [
        "",
        "RAG Pipeline Health Check",
        "=" * 60,
        "",
        f"{'Component':<20} {'Status':<10} {'Details'}",
        "-" * 60,
    ]

    # Qdrant connection
    conn_status = "[OK]" if checks['qdrant_connected'] else "[FAIL]"
    lines.append(f"{'Qdrant Connection':<20} {conn_status:<10} {checks['qdrant_message']}")

    # Collection status
    if checks['qdrant_connected']:
        coll_status = "[OK]" if checks['collection_exists'] else "[FAIL]"
        coll_details = checks['collection_name'] if checks['collection_exists'] else checks['collection_error']
        lines.append(f"{'Collection':<20} {coll_status:<10} {coll_details}")

        # Vector count
        if checks['collection_exists']:
            lines.append(f"{'Vector Count':<20} {'[OK]':<10} {checks['vector_count']:,} vectors")
        else:
            lines.append(f"{'Vector Count':<20} {'[SKIP]':<10} (requires collection)")

        # Sample query
        if checks['collection_exists'] and checks['vector_count'] > 0:
            query_status = "[OK]" if checks['sample_query_success'] else "[FAIL]"
            query_details = f"{checks['sample_query_count']} results in {checks['sample_query_time']:.0f}ms"
            lines.append(f"{'Sample Query':<20} {query_status:<10} {query_details}")
        else:
            lines.append(f"{'Sample Query':<20} {'[SKIP]':<10} (requires vectors)")
    else:
        lines.append(f"{'Collection':<20} {'[SKIP]':<10} (requires connection)")
        lines.append(f"{'Vector Count':<20} {'[SKIP]':<10} (requires connection)")
        lines.append(f"{'Sample Query':<20} {'[SKIP]':<10} (requires connection)")

    lines.append("")
    lines.append("=" * 60)

    # Overall status
    is_healthy = (
        checks['qdrant_connected'] and
        checks['collection_exists'] and
        checks['vector_count'] > 0
    )
    overall = "HEALTHY" if is_healthy else "UNHEALTHY"
    lines.append(f"Overall Status: {overall}")

    # Add errors if any
    if checks.get('errors'):
        lines.append("")
        lines.append("Errors:")
        for error in checks['errors']:
            lines.append(f"  - {error}")

    return "\n".join(lines)


def health_check() -> int:
    """Run comprehensive health check on the RAG pipeline.

    Returns:
        Exit code (0=healthy, 1=config error, 2=unhealthy)
    """
    checks = {
        'qdrant_connected': False,
        'qdrant_message': '',
        'collection_exists': False,
        'collection_name': '',
        'collection_error': '',
        'vector_count': 0,
        'sample_query_success': False,
        'sample_query_time': 0.0,
        'sample_query_count': 0,
        'errors': []
    }

    # Validate environment
    is_valid, missing = validate_env()
    if not is_valid:
        checks['errors'].append(f"Missing environment variables: {', '.join(missing)}")
        print(format_health_report(checks))
        return 1

    # Initialize clients
    cohere_client = get_cohere_client()
    if not cohere_client:
        checks['errors'].append("Failed to initialize Cohere client")

    qdrant_client = get_qdrant_client()
    if not qdrant_client:
        checks['qdrant_message'] = "Connection failed"
        checks['errors'].append("Failed to connect to Qdrant")
        print(format_health_report(checks))
        return 2

    # Check Qdrant connection
    connected, message = check_qdrant_connection(qdrant_client)
    checks['qdrant_connected'] = connected
    checks['qdrant_message'] = message

    if not connected:
        checks['errors'].append(f"Qdrant connection: {message}")
        print(format_health_report(checks))
        return 2

    # Check collection
    exists, name_or_error, count = check_collection_exists(qdrant_client)
    checks['collection_exists'] = exists
    if exists:
        checks['collection_name'] = name_or_error
        checks['vector_count'] = count
    else:
        checks['collection_error'] = name_or_error
        checks['errors'].append(f"Collection '{COLLECTION_NAME}' not found. Run main.py first to index documents.")

    # Run sample query if collection has vectors
    if exists and count > 0 and cohere_client:
        success, time_ms, result_count = run_sample_query(qdrant_client, cohere_client)
        checks['sample_query_success'] = success
        checks['sample_query_time'] = time_ms
        checks['sample_query_count'] = result_count

        if not success:
            checks['errors'].append("Sample query failed")

    # Display report
    print(format_health_report(checks))

    # Return status
    is_healthy = (
        checks['qdrant_connected'] and
        checks['collection_exists'] and
        checks['vector_count'] > 0
    )
    return 0 if is_healthy else 2


def main() -> int:
    """Main entry point for the retrieval CLI.

    Returns:
        Exit code
    """
    parser = argparse.ArgumentParser(
        description="RAG Retrieval Pipeline Testing Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python retrieve.py "ROS 2 nodes"              # Basic search
  python retrieve.py "robot navigation" --k 10  # Get 10 results
  python retrieve.py "Isaac Sim" --threshold 0.5  # Filter by score
  python retrieve.py --health                   # Run health check
        """
    )

    parser.add_argument(
        'query',
        nargs='?',
        help='Search query text'
    )
    parser.add_argument(
        '--k',
        type=int,
        default=DEFAULT_K,
        help=f'Number of results to return (default: {DEFAULT_K})'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=DEFAULT_THRESHOLD,
        help=f'Minimum similarity score 0.0-1.0 (default: {DEFAULT_THRESHOLD})'
    )
    parser.add_argument(
        '--health',
        action='store_true',
        help='Run health check instead of search'
    )

    args = parser.parse_args()

    # Handle health check mode
    if args.health:
        return health_check()

    # Require query for search mode
    if not args.query:
        parser.print_help()
        print("\nError: Query text is required for search. Use --health for health check.")
        return 3

    # Validate threshold
    if not 0.0 <= args.threshold <= 1.0:
        print("Error: Threshold must be between 0.0 and 1.0")
        return 3

    # Validate k
    if args.k < 1 or args.k > 100:
        print("Error: k must be between 1 and 100")
        return 3

    # Execute search
    return search(args.query, args.k, args.threshold)


if __name__ == "__main__":
    sys.exit(main())
