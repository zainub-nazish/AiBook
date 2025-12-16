"""
RAG Retrieval-Enabled Agent for Physical AI & Robotics Documentation

This module provides a conversational agent that retrieves relevant documentation
from Qdrant and generates grounded answers with source citations using OpenAI.

Usage:
    python agent.py "your question"              # Single query mode
    python agent.py                              # Interactive mode
    python agent.py "question" --debug           # Show debug info
    python agent.py "question" --no-citations    # Hide sources
"""

import argparse
import json
import logging
import os
import sys

from dotenv import load_dotenv
from openai import OpenAI

# Import retrieval functions from retrieve.py
from retrieve import (
    get_qdrant_client,
    get_cohere_client,
    embed_query,
    search_similar,
    sanitize_text
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Constants
MODEL = "gpt-4o-mini"
MAX_QUERY_LENGTH = 8000
COLLECTION_NAME = "rag_embedding"
DEFAULT_K = 5
DEFAULT_THRESHOLD = 0.0
MAX_HISTORY = 10

# System prompt for grounded responses
SYSTEM_PROMPT = """You are a helpful assistant that answers questions about Physical AI and Robotics based on the documentation provided.

CRITICAL RULES:
1. You MUST ONLY answer based on the context provided by the retrieve_documentation tool
2. If no relevant information is found, you MUST say: "I couldn't find relevant information about that topic in the documentation."
3. NEVER make up information or use knowledge outside the provided context
4. Always include source citations with URLs when answering
5. If the retrieved context is only partially relevant, clearly state what you found and what you couldn't find

When citing sources, format them as:
- [Title] (relevance: XX%)
  URL: https://...

Be concise but thorough in your answers."""

# OpenAI function tool definition
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "retrieve_documentation",
            "description": "Search the Physical AI & Robotics documentation for relevant information about a topic. Use this tool to find context before answering any question.",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "The search query to find relevant documentation"
                    }
                },
                "required": ["query"]
            }
        }
    }
]


def validate_env() -> tuple[bool, list[str]]:
    """Validate required environment variables are present.

    Returns:
        Tuple of (is_valid, list of missing variable names)
    """
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'OPENAI_API_KEY']
    missing = [var for var in required_vars if not os.getenv(var)]
    return (len(missing) == 0, missing)


def retrieve_documentation(
    query: str,
    cohere_client,
    qdrant_client,
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD,
    debug: bool = False
) -> list[dict]:
    """Retrieve relevant documentation chunks from Qdrant.

    Args:
        query: Search query text
        cohere_client: Initialized Cohere client
        qdrant_client: Initialized Qdrant client
        k: Number of results to return
        threshold: Minimum similarity score
        debug: Whether to print debug info

    Returns:
        List of result dicts with text, url, title, score
    """
    if debug:
        print(f"[DEBUG] Generating embedding for query: {query[:50]}...")

    # Generate query embedding
    query_vector = embed_query(query, cohere_client)
    if not query_vector:
        logger.error("Failed to generate query embedding")
        return []

    if debug:
        print(f"[DEBUG] Searching Qdrant with k={k}, threshold={threshold}")

    # Search Qdrant
    results = search_similar(qdrant_client, query_vector, k=k, threshold=threshold)

    if debug:
        print(f"[DEBUG] Retrieved {len(results)} chunks:")
        for i, r in enumerate(results, 1):
            print(f"  - Chunk {i}: score={r['score']:.3f}, title=\"{sanitize_text(r['title'])[:40]}\"")

    return results


def format_sources(results: list[dict], show_citations: bool = True) -> str:
    """Format retrieval results as source citations.

    Args:
        results: List of retrieval result dicts
        show_citations: Whether to include citations

    Returns:
        Formatted string with sources
    """
    if not show_citations or not results:
        return ""

    lines = ["\nSources:"]
    for r in results:
        score_percent = r['score'] * 100
        title = sanitize_text(r['title'])
        lines.append(f"- {title} (relevance: {score_percent:.1f}%)")
        lines.append(f"  URL: {r['url']}")

    return "\n".join(lines)


def format_context_for_agent(results: list[dict]) -> str:
    """Format retrieval results as context for the agent.

    Args:
        results: List of retrieval result dicts

    Returns:
        Formatted context string
    """
    if not results:
        return "No relevant documentation found."

    context_parts = []
    for i, r in enumerate(results, 1):
        text = sanitize_text(r['text'])
        title = sanitize_text(r['title'])
        context_parts.append(f"[Source {i}] {title}\nURL: {r['url']}\n{text}")

    return "\n\n---\n\n".join(context_parts)


def process_query(
    query: str,
    messages: list[dict],
    openai_client: OpenAI,
    cohere_client,
    qdrant_client,
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD,
    debug: bool = False,
    show_citations: bool = True
) -> tuple[str, list[dict]]:
    """Process a single query through the agent.

    Args:
        query: User's question
        messages: Conversation history
        openai_client: OpenAI client
        cohere_client: Cohere client for embeddings
        qdrant_client: Qdrant client for retrieval
        k: Number of chunks to retrieve
        threshold: Minimum similarity score
        debug: Show debug output
        show_citations: Include source citations

    Returns:
        Tuple of (answer text, retrieval results)
    """
    # Add user message
    messages.append({"role": "user", "content": query})

    retrieval_results = []

    # Call OpenAI with tools
    try:
        if debug:
            print(f"[DEBUG] Sending query to OpenAI with tools enabled")

        response = openai_client.chat.completions.create(
            model=MODEL,
            messages=messages,
            tools=TOOLS,
            tool_choice="auto"
        )

        assistant_message = response.choices[0].message

        # Handle tool calls
        if assistant_message.tool_calls:
            # Process each tool call
            tool_messages = []

            for tool_call in assistant_message.tool_calls:
                if tool_call.function.name == "retrieve_documentation":
                    # Parse arguments
                    args = json.loads(tool_call.function.arguments)
                    search_query = args.get("query", query)

                    if debug:
                        print(f"[DEBUG] Agent requested retrieval for: {search_query}")

                    # Execute retrieval
                    retrieval_results = retrieve_documentation(
                        search_query,
                        cohere_client,
                        qdrant_client,
                        k=k,
                        threshold=threshold,
                        debug=debug
                    )

                    # Format context for agent
                    context = format_context_for_agent(retrieval_results)

                    tool_messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": context
                    })

            # Add assistant message with tool calls
            messages.append({
                "role": "assistant",
                "content": assistant_message.content,
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {
                            "name": tc.function.name,
                            "arguments": tc.function.arguments
                        }
                    }
                    for tc in assistant_message.tool_calls
                ]
            })

            # Add tool responses
            messages.extend(tool_messages)

            # Get final response from agent
            if debug:
                print(f"[DEBUG] Getting final response from agent with {len(retrieval_results)} context chunks")

            final_response = openai_client.chat.completions.create(
                model=MODEL,
                messages=messages
            )

            answer = final_response.choices[0].message.content
            messages.append({"role": "assistant", "content": answer})

        else:
            # No tool call - agent responded directly
            answer = assistant_message.content or "I need to search the documentation to answer that question."
            messages.append({"role": "assistant", "content": answer})

        # Add source citations if available
        if show_citations and retrieval_results:
            sources = format_sources(retrieval_results, show_citations=True)
            return answer + sources, retrieval_results

        return answer, retrieval_results

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        error_msg = f"Error: Failed to process query. {str(e)}"
        return error_msg, []


def single_query_mode(
    query: str,
    openai_client: OpenAI,
    cohere_client,
    qdrant_client,
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD,
    debug: bool = False,
    show_citations: bool = True
) -> int:
    """Execute a single query and exit.

    Args:
        query: User's question
        openai_client: OpenAI client
        cohere_client: Cohere client
        qdrant_client: Qdrant client
        k: Number of chunks to retrieve
        threshold: Minimum similarity
        debug: Show debug output
        show_citations: Include sources

    Returns:
        Exit code
    """
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    answer, results = process_query(
        query,
        messages,
        openai_client,
        cohere_client,
        qdrant_client,
        k=k,
        threshold=threshold,
        debug=debug,
        show_citations=show_citations
    )

    print(f"\nAnswer:\n{answer}")
    return 0


class ConversationContext:
    """Manages conversation history for multi-turn interactions."""

    def __init__(self, max_history: int = MAX_HISTORY):
        """Initialize conversation context.

        Args:
            max_history: Maximum number of messages to retain
        """
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        self.max_history = max_history

    def add_message(self, role: str, content: str):
        """Add a message to history.

        Args:
            role: Message role (user/assistant)
            content: Message content
        """
        self.messages.append({"role": role, "content": content})
        self._truncate()

    def _truncate(self):
        """Truncate history if exceeds max_history."""
        # Keep system message + last max_history messages
        if len(self.messages) > self.max_history + 1:
            self.messages = [self.messages[0]] + self.messages[-(self.max_history):]

    def clear(self):
        """Clear conversation history (keep system prompt)."""
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    def get_messages(self) -> list[dict]:
        """Get current message history."""
        return self.messages.copy()


def show_help():
    """Display help message for interactive mode."""
    print("""
Available commands:
  exit, quit  - End the conversation
  clear       - Clear conversation history
  help        - Show this help message

Tips:
  - Ask questions about Physical AI, Robotics, ROS 2, Isaac Sim, etc.
  - Follow-up questions maintain context from previous exchanges
  - The agent only answers based on indexed documentation
""")


def interactive_mode(
    openai_client: OpenAI,
    cohere_client,
    qdrant_client,
    k: int = DEFAULT_K,
    threshold: float = DEFAULT_THRESHOLD,
    debug: bool = False,
    show_citations: bool = True
) -> int:
    """Run interactive conversation mode.

    Args:
        openai_client: OpenAI client
        cohere_client: Cohere client
        qdrant_client: Qdrant client
        k: Number of chunks to retrieve
        threshold: Minimum similarity
        debug: Show debug output
        show_citations: Include sources

    Returns:
        Exit code
    """
    print("\nRAG Agent - Physical AI & Robotics Documentation")
    print("Type 'exit' to quit, 'help' for commands.\n")

    context = ConversationContext()

    while True:
        try:
            user_input = input("You: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nGoodbye!")
            return 0

        if not user_input:
            continue

        # Handle special commands
        cmd = user_input.lower()
        if cmd in ["exit", "quit"]:
            print("Goodbye!")
            return 0
        elif cmd == "clear":
            context.clear()
            print("Conversation history cleared.\n")
            continue
        elif cmd == "help":
            show_help()
            continue

        # Validate input
        if len(user_input) > MAX_QUERY_LENGTH:
            print(f"Error: Query too long. Maximum {MAX_QUERY_LENGTH} characters.\n")
            continue

        # Process query
        messages = context.get_messages()
        answer, results = process_query(
            user_input,
            messages,
            openai_client,
            cohere_client,
            qdrant_client,
            k=k,
            threshold=threshold,
            debug=debug,
            show_citations=show_citations
        )

        # Update context with the conversation
        context.messages = messages

        print(f"\nAnswer:\n{answer}\n")


def main() -> int:
    """Main entry point for the agent CLI.

    Returns:
        Exit code
    """
    parser = argparse.ArgumentParser(
        description="RAG Agent for Physical AI & Robotics Documentation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python agent.py "What is ROS 2?"              # Single query
  python agent.py                               # Interactive mode
  python agent.py "Isaac Sim" --k 10            # Get 10 chunks
  python agent.py "URDF" --debug                # Show debug info
  python agent.py "navigation" --no-citations   # Hide sources
        """
    )

    parser.add_argument(
        'query',
        nargs='?',
        help='Question to ask (interactive mode if not provided)'
    )
    parser.add_argument(
        '--k',
        type=int,
        default=DEFAULT_K,
        help=f'Number of chunks to retrieve (default: {DEFAULT_K})'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=DEFAULT_THRESHOLD,
        help=f'Minimum similarity score 0.0-1.0 (default: {DEFAULT_THRESHOLD})'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Show debug logging'
    )
    parser.add_argument(
        '--no-citations',
        action='store_true',
        help='Suppress source citations'
    )

    args = parser.parse_args()

    # Validate environment
    is_valid, missing = validate_env()
    if not is_valid:
        print(f"Error: Missing required environment variables: {', '.join(missing)}")
        print("Please add them to your .env file.")
        return 1

    # Validate parameters
    if not 0.0 <= args.threshold <= 1.0:
        print("Error: Threshold must be between 0.0 and 1.0")
        return 3

    if args.k < 1 or args.k > 100:
        print("Error: k must be between 1 and 100")
        return 3

    # Initialize clients
    try:
        openai_client = OpenAI()
    except Exception as e:
        print(f"Error: Failed to initialize OpenAI client: {e}")
        return 1

    cohere_client = get_cohere_client()
    if not cohere_client:
        print("Error: Failed to initialize Cohere client. Check COHERE_API_KEY.")
        return 1

    qdrant_client = get_qdrant_client()
    if not qdrant_client:
        print("Error: Cannot connect to Qdrant. Check QDRANT_URL and QDRANT_API_KEY.")
        return 2

    show_citations = not args.no_citations

    # Execute mode
    if args.query:
        # Validate query
        query = args.query.strip()
        if not query:
            print("Error: Query text cannot be empty.")
            return 3

        if len(query) > MAX_QUERY_LENGTH:
            print(f"Error: Query too long. Maximum {MAX_QUERY_LENGTH} characters.")
            return 3

        return single_query_mode(
            query,
            openai_client,
            cohere_client,
            qdrant_client,
            k=args.k,
            threshold=args.threshold,
            debug=args.debug,
            show_citations=show_citations
        )
    else:
        return interactive_mode(
            openai_client,
            cohere_client,
            qdrant_client,
            k=args.k,
            threshold=args.threshold,
            debug=args.debug,
            show_citations=show_citations
        )


if __name__ == "__main__":
    sys.exit(main())
