# CLI Interface Contract: RAG Retrieval Agent

**Feature**: 007-rag-retrieval-agent
**Date**: 2025-12-16

## Command Line Interface

### Interactive Mode (Default)

```bash
python agent.py
```

Starts an interactive conversation loop where the user can ask multiple questions.

**Behavior**:
- Display welcome message with instructions
- Enter REPL (Read-Eval-Print-Loop)
- Accept user input via stdin
- Process question and display answer
- Continue until user types "exit" or "quit"

### Single Query Mode

```bash
python agent.py "What is Isaac Sim?"
```

Ask a single question and exit.

**Arguments**:
| Argument | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| query | string | No | None | Question text (interactive mode if not provided) |

**Options**:
| Flag | Type | Default | Description |
|------|------|---------|-------------|
| --k | int | 5 | Number of chunks to retrieve |
| --threshold | float | 0.0 | Minimum similarity score |
| --no-citations | flag | false | Suppress source citations in output |
| --debug | flag | false | Show debug logging (chunks retrieved, scores) |

## Input Specification

### Question Format
- **Type**: String
- **Min Length**: 1 character (after stripping whitespace)
- **Max Length**: 8000 characters
- **Encoding**: UTF-8

### Special Commands (Interactive Mode)
| Command | Action |
|---------|--------|
| exit | End conversation and exit |
| quit | End conversation and exit |
| clear | Clear conversation history |
| help | Show help message |

## Output Specification

### Success Response Format

```text
Answer:
[Generated answer text based on retrieved content]

Sources:
- [Title 1] (score: 85.2%)
  URL: https://example.com/docs/page1
- [Title 2] (score: 72.1%)
  URL: https://example.com/docs/page2
```

### No Information Response

```text
I couldn't find relevant information about that topic in the documentation.
Please try rephrasing your question or asking about a different topic covered
in the Physical AI & Robotics book.
```

### Error Response Format

```text
Error: [Error message]
Details: [Additional context if available]
```

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Configuration error (missing env vars, API keys) |
| 2 | Connection error (Qdrant unreachable, API timeout) |
| 3 | Input validation error (empty query, too long) |
| 4 | Runtime error (unexpected failure) |

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| COHERE_API_KEY | Yes | Cohere API key for query embeddings |
| QDRANT_URL | Yes | Qdrant server URL |
| QDRANT_API_KEY | Yes | Qdrant authentication key |
| OPENAI_API_KEY | Yes | OpenAI API key for agent LLM |

## Examples

### Basic Query
```bash
$ python agent.py "What is ROS 2?"

Answer:
ROS 2 (Robot Operating System 2) is an open-source middleware framework for
building robot applications. It provides hardware abstraction, device drivers,
communication between processes, and package management.

Sources:
- Introduction to ROS 2 (score: 89.3%)
  URL: https://physicalai-robotics.com/docs/ros2/intro
```

### Interactive Session
```bash
$ python agent.py

RAG Agent - Physical AI & Robotics Documentation
Type 'exit' to quit, 'help' for commands.

You: What is Isaac Sim?

Answer:
Isaac Sim is NVIDIA's robot simulation platform built on Omniverse...

Sources:
- Isaac Sim Overview (score: 91.2%)
  URL: https://physicalai-robotics.com/docs/isaac-sim/overview

You: How do I install it?

Answer:
To install Isaac Sim, you need to first install NVIDIA Omniverse...
[Context-aware follow-up answer]

You: exit
Goodbye!
```

### Debug Mode
```bash
$ python agent.py "What is URDF?" --debug

[DEBUG] Generating embedding for query...
[DEBUG] Searching Qdrant with k=5, threshold=0.0
[DEBUG] Retrieved 5 chunks:
  - Chunk 1: score=0.872, title="URDF Basics"
  - Chunk 2: score=0.834, title="Robot Description Formats"
  ...
[DEBUG] Sending to agent with 5 context chunks

Answer:
URDF (Unified Robot Description Format) is an XML format for describing...

Sources:
- URDF Basics (score: 87.2%)
  URL: https://physicalai-robotics.com/docs/formats/urdf
```

## Error Handling

### Missing API Key
```bash
$ python agent.py "test"
Error: Missing required environment variable: OPENAI_API_KEY
Details: Please add OPENAI_API_KEY to your .env file
Exit code: 1
```

### Qdrant Unreachable
```bash
$ python agent.py "test"
Error: Cannot connect to Qdrant
Details: Check QDRANT_URL (https://xxx.qdrant.cloud) is accessible
Exit code: 2
```

### Empty Query
```bash
$ python agent.py ""
Error: Query text cannot be empty
Exit code: 3
```
