# RAG Chatbot Backend

Retrieval-Augmented Generation (RAG) chatbot backend for Physical AI & Humanoid Robotics textbook.

## Setup

1. Install dependencies:
   ```bash
   uv sync
   ```

2. Create `.env` file from template:
   ```bash
   cp .env.example .env
   ```

3. Edit `.env` with your API keys:
   - Cohere API: https://cohere.com/
   - Qdrant Cloud: https://cloud.qdrant.io/
   - OpenRouter: https://openrouter.ai/

4. Run scraper to ingest documentation (first time only):
   ```bash
   uv run python scrapper.py
   ```

5. Start server:
   ```bash
   uv run python main.py
   ```

## API Endpoints

- `GET /` - API status
- `POST /chat` - Chat with RAG pipeline
- `GET /health` - Service health check
- `GET /collection-stats` - Qdrant collection statistics

## Testing

Run test suite:
```bash
uv run python retrieving.py
```

Test custom query:
```bash
uv run python retrieving.py "what is ROS2?"
```

Debug mode:
```bash
uv run python retrieving.py debug hardware
```