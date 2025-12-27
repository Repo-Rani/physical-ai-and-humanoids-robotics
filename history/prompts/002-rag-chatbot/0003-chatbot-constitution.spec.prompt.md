RAG Chatbot Integration Constitution
Project Overview
Implement a complete RAG (Retrieval-Augmented Generation) chatbot system for a Docusaurus-based Physical AI & Humanoid Robotics textbook with advanced features including persistent chat history, text selection query, and enhanced UI.

Constitution Breakdown
CONSTITUTION 1: Backend Setup & Infrastructure
File: backend-setup.md
markdown# Backend Infrastructure Constitution

## Objective
Set up a complete FastAPI backend with UV project management, Qdrant vector database integration, and RAG pipeline.

## Folder Structure
````
backend/
├── main.py                 # FastAPI server with all endpoints
├── scrapper.py            # Web scraper for Qdrant ingestion
├── retrieving.py          # RAG testing and debugging tool
├── agent.py               # OpenAI Agents-based RAG agent
├── .env.example           # Environment variables template
├── .env                   # Actual environment variables (gitignored)
├── pyproject.toml         # UV project configuration
├── README.md              # Setup instructions
└── .gitignore             # Git ignore file
````

## Step 1: Initialize UV Project
````bash
cd backend
uv init
````

## Step 2: Create pyproject.toml
File: `backend/pyproject.toml`
````toml
[project]
name = "backend"
version = "0.1.0"
description = "RAG Chatbot Backend for Physical AI & Humanoid Robotics"
readme = "README.md"
requires-python = ">=3.11"
dependencies = [
    "beautifulsoup4>=4.14.3",
    "cohere>=5.20.1",
    "openai-agents>=0.6.4",
    "python-dotenv>=1.2.1",
    "qdrant-client>=1.16.2",
    "requests>=2.32.5",
    "selenium>=4.39.0",
    "fastapi>=0.115.0",
    "uvicorn>=0.32.0",
]
````

## Step 3: Create .env.example
File: `backend/.env.example`
````env
# Cohere API for embeddings
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COLLECTION_NAME=physical_ai_humanoids_robotics

# OpenRouter API for LLM
OPENROUTER_API_KEY=your_openrouter_api_key_here
````

## Step 4: Create .gitignore
File: `backend/.gitignore`
````
.env
__pycache__/
*.pyc
.uv/
.venv/
venv/
*.log
````

## Step 5: Create README.md
File: `backend/README.md`
````markdown
# RAG Chatbot Backend

## Setup Instructions

1. **Install UV** (if not installed):
```bash
   curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. **Clone and navigate**:
```bash
   cd backend
```

3. **Install dependencies**:
```bash
   uv sync
```

4. **Configure environment**:
   - Copy `.env.example` to `.env`
   - Fill in your API keys

5. **Ingest documentation** (first time only):
```bash
   uv run python scrapper.py
```

6. **Start backend server**:
```bash
   uv run python main.py
```

7. **Test retrieval** (optional):
```bash
   uv run python retrieving.py
```

## Endpoints
- `GET /` - API info
- `POST /chat` - Main chat endpoint
- `GET /health` - Health check
- `GET /collection-stats` - Qdrant statistics
````

## Validation Criteria
- ✅ UV project initializes without errors
- ✅ All dependencies install correctly
- ✅ .env file structure matches requirements
- ✅ README provides clear setup instructions

CONSTITUTION 2: FastAPI Server Implementation
File: fastapi-server.md
markdown# FastAPI Server Constitution

## Objective
Create a production-ready FastAPI server with RAG endpoints, CORS configuration, and error handling.

## File: `backend/main.py`

### Requirements
1. FastAPI application with CORS middleware
2. Cohere embeddings integration
3. Qdrant vector database queries
4. OpenRouter LLM integration with fallback models
5. Persistent conversation support
6. Health check endpoints

### Complete Implementation
````python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
import requests

load_dotenv()

app = FastAPI(title="Humanoid Robotics Chatbot API")

# CORS setup - allows frontend to communicate
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production: specify exact domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_humanoids_robotics")

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
if not OPENROUTER_API_KEY:
    raise ValueError("OPENROUTER_API_KEY not found in environment variables")

OPENROUTER_API_URL = "https://openrouter.ai/api/v1/chat/completions"

# Free models with fallback
OPENROUTER_MODELS = [
    "meta-llama/llama-3.2-3b-instruct:free",
    "google/gemma-2-9b-it:free", 
    "mistralai/mistral-7b-instruct:free",
    "nousresearch/hermes-3-llama-3.1-405b:free"
]

cohere_client = cohere.Client(COHERE_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

def get_embedding(text: str):
    """Generate embedding using Cohere"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

def retrieve_documents(query: str, limit: int = 5):
    """Retrieve relevant documents from Qdrant"""
    try:
        embedding = get_embedding(query)
        
        result = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=limit,
            with_payload=True
        )
        
        documents = []
        for point in result.points:
            documents.append({
                "text": point.payload.get("text", ""),
                "url": point.payload.get("url", ""),
                "title": point.payload.get("page_title", "Unknown"),
                "score": point.score
            })
        
        return documents
    except Exception as e:
        print(f"Error retrieving documents: {e}")
        return []

def generate_response(query: str, documents: list, model_index: int = 0):
    """Generate response using OpenRouter with fallback"""
    
    if not documents:
        return "I apologize, but I couldn't find relevant information in the documentation. Could you please rephrase your question?"
    
    context = "\n\n".join([
        f"Document {i+1} (from {doc['title']}):\n{doc['text'][:800]}"
        for i, doc in enumerate(documents[:3])
    ])
    
    system_prompt = """You are an AI tutor for Physical AI & Humanoid Robotics. Answer the user's question based ONLY on the provided documentation.

INSTRUCTIONS:
1. Answer based ONLY on the provided documentation
2. If the answer isn't in the documentation, say "I don't have information about this in the current documentation"
3. Keep your answer concise (2-3 paragraphs)
4. Use emojis occasionally to make it engaging
5. Mention which section/module the information came from
6. Be helpful and friendly"""

    user_prompt = f"""DOCUMENTATION CONTEXT:
{context}

USER QUESTION:
{query}

Your answer:"""
    
    # Try models in order until one works
    for idx in range(model_index, len(OPENROUTER_MODELS)):
        current_model = OPENROUTER_MODELS[idx]
        
        try:
            headers = {
                "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                "Content-Type": "application/json",
                "HTTP-Referer": "http://localhost:8000",
                "X-Title": "Humanoid Robotics Chatbot"
            }
            
            payload = {
                "model": current_model,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                "temperature": 0.7,
                "max_tokens": 800
            }
            
            print(f"Trying model {idx + 1}/{len(OPENROUTER_MODELS)}: {current_model}")
            
            response = requests.post(
                OPENROUTER_API_URL,
                headers=headers,
                json=payload,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                
                if "choices" not in result or len(result["choices"]) == 0:
                    continue
                
                choice = result["choices"][0]
                
                if "error" in choice:
                    error_code = choice["error"].get("code", "N/A")
                    if error_code in [502, 503]:
                        continue
                
                content = choice.get("message", {}).get("content", "").strip()
                
                if content:
                    print(f"✅ Success with model: {current_model}")
                    return content
                else:
                    continue
                    
            elif response.status_code == 429:
                continue
            else:
                continue
                
        except requests.exceptions.Timeout:
            continue
        except Exception as e:
            print(f"Error with {current_model}: {e}")
            continue
    
    return "I'm experiencing technical difficulties with all available models. Please try again in a moment."

class ChatRequest(BaseModel):
    message: str
    conversation_history: list = []

class ChatResponse(BaseModel):
    response: str
    success: bool
    sources: list = []

@app.get("/")
async def root():
    return {
        "message": "🤖 Humanoid Robotics Chatbot API",
        "status": "running",
        "endpoints": {
            "/chat": "POST - Send messages",
            "/health": "GET - Check system health",
            "/docs": "GET - API documentation"
        }
    }

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Main chat endpoint"""
    try:
        documents = retrieve_documents(request.message, limit=5)
        
        if not documents:
            return ChatResponse(
                response="I couldn't find relevant information in the documentation. Please try rephrasing your question or ask about topics covered in our Physical AI & Humanoid Robotics modules.",
                success=True,
                sources=[]
            )
        
        ai_response = generate_response(request.message, documents)
        
        sources = [
            {
                "title": doc["title"],
                "url": doc["url"],
                "score": round(doc["score"], 3)
            }
            for doc in documents[:3]
        ]
        
        return ChatResponse(
            response=ai_response,
            success=True,
            sources=sources
        )
    
    except Exception as e:
        print(f"Chat error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing request: {str(e)}"
        )

@app.get("/health")
async def health_check():
    """Check system health"""
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        test_embedding = get_embedding("test")
        
        test_headers = {
            "Authorization": f"Bearer {OPENROUTER_API_KEY}",
            "Content-Type": "application/json"
        }
        test_response = requests.get(
            "https://openrouter.ai/api/v1/models",
            headers=test_headers,
            timeout=5
        )
        openrouter_status = "✅ Connected" if test_response.status_code == 200 else "❌ Error"
        
        return {
            "status": "healthy",
            "services": {
                "qdrant": "✅ Connected",
                "cohere": "✅ Connected",
                "openrouter": openrouter_status
            },
            "collection": {
                "name": COLLECTION_NAME,
                "documents": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size
            }
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }

@app.get("/collection-stats")
async def collection_stats():
    """Get collection statistics"""
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        
        return {
            "collection_name": COLLECTION_NAME,
            "total_documents": collection_info.points_count,
            "vector_dimensions": collection_info.config.params.vectors.size,
            "distance_metric": collection_info.config.params.vectors.distance
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    print("\n" + "="*60)
    print("🚀 Starting Humanoid Robotics Chatbot API")
    print("="*60)
    print("📍 API: http://localhost:8000")
    print("📚 Docs: http://localhost:8000/docs")
    print("❤️  Health: http://localhost:8000/health")
    print("="*60 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000)
````

## Validation Criteria
- ✅ Server starts on port 8000
- ✅ CORS allows frontend requests
- ✅ Health endpoint returns service status
- ✅ Chat endpoint returns AI responses
- ✅ Error handling works correctly

CONSTITUTION 3: Web Scraper for Qdrant Ingestion
File: web-scraper.md
markdown# Web Scraper Constitution

## Objective
Create a robust web scraper that extracts content from Docusaurus documentation and ingests it into Qdrant with proper chunking and embeddings.

## File: `backend/scrapper.py`

### Complete Implementation
````python
"""
Web Scraper for Physical AI & Humanoid Robotics Documentation
Run this file separately to ingest/update documentation in Qdrant
"""

import requests
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from bs4 import BeautifulSoup
import time
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import os
from dotenv import load_dotenv

load_dotenv()

BASE_URL = "https://physical-ai-and-humanoids-robotics.vercel.app"
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_humanoids_robotics")

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

EMBED_MODEL = "embed-english-v3.0"
VECTOR_SIZE = 1024

cohere_client = cohere.Client(COHERE_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

URLS_TO_SCRAPE = [
    f"{BASE_URL}/docs/intro",
    f"{BASE_URL}/docs/hardware-guide",
    f"{BASE_URL}/docs/instructor-guide",
    f"{BASE_URL}/docs/glossary",
    f"{BASE_URL}/docs/changelog",
    
    # Module 0 - Getting Started
    f"{BASE_URL}/docs/module-0-getting-started/",
    f"{BASE_URL}/docs/module-0-getting-started/module-0-chapter-1",
    f"{BASE_URL}/docs/module-0-getting-started/module-0-chapter-2",
    f"{BASE_URL}/docs/module-0-getting-started/module-0-chapter-3",
    
    # Module 1 - ROS2
    f"{BASE_URL}/docs/module-1-ros2/",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-1",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-2",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-3",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-4",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-5",
    f"{BASE_URL}/docs/module-1-ros2/module-1-chapter-6",
    
    # Module 2 - Digital Twin/Simulation
    f"{BASE_URL}/docs/module-2-digital-twin/",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-1",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-2",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-3",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-4",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-5",
    f"{BASE_URL}/docs/module-2-digital-twin/module-2-chapter-6",
    
    # Module 3 - Isaac
    f"{BASE_URL}/docs/module-3-isaac/",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-1",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-2",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-3",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-4",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-5",
    f"{BASE_URL}/docs/module-3-isaac/module-3-chapter-6",
    
    # Module 4 - VLA
    f"{BASE_URL}/docs/module-4-vla/",
    f"{BASE_URL}/docs/module-4-vla/module-4-chapter-1",
    f"{BASE_URL}/docs/module-4-vla/module-4-chapter-2",
    f"{BASE_URL}/docs/module-4-vla/module-4-chapter-3",
    f"{BASE_URL}/docs/module-4-vla/module-4-chapter-4",
    f"{BASE_URL}/docs/module-4-vla/module-4-chapter-5",
    
    # Module 5 - Capstone
    f"{BASE_URL}/docs/module-5-capstone/",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-1",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-2",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-3",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-4",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-5",
    f"{BASE_URL}/docs/module-5-capstone/module-5-chapter-6",
    
    # Additional Pages
    f"{BASE_URL}/markdown-page",
]

def setup_driver():
    chrome_options = Options()
    chrome_options.add_argument('--headless')
    chrome_options.add_argument('--no-sandbox')
    chrome_options.add_argument('--disable-dev-shm-usage')
    chrome_options.add_argument('--disable-blink-features=AutomationControlled')
    chrome_options.add_argument('user-agent=Mozilla/5.0 (Windows NT 10.0; Win64; x64)')
    return webdriver.Chrome(options=chrome_options)

def extract_page_title(soup):
    """Extract page title"""
    title = soup.find('h1')
    if title:
        return title.get_text(strip=True)
    
    meta_title = soup.find('meta', property='og:title')
    if meta_title:
        return meta_title.get('content', 'Unknown')
    
    return "Unknown"

def extract_text_from_url(url):
    """Extract text from URL using Selenium"""
    driver = setup_driver()
    try:
        driver.get(url)
        time.sleep(2)
        
        html = driver.page_source
        soup = BeautifulSoup(html, 'html.parser')
        
        page_title = extract_page_title(soup)
        
        for element in soup(["script", "style", "nav", "footer", "header", "aside", "button"]):
            element.decompose()
        
        for code in soup.find_all('pre'):
            code_text = code.get_text(strip=True)
            if len(code_text) > 1000:
                code.decompose()
        
        for meta in soup.find_all(class_=['reading-time', 'metadata', 'author-info']):
            meta.decompose()
        
        main_content = (
            soup.find('article') or 
            soup.find('main') or 
            soup.find(class_='markdown') or
            soup.find('div', class_='content') or
            soup.find('body')
        )
        
        if main_content:
            text = main_content.get_text(separator=' ', strip=True)
            text = ' '.join(text.split())
            text = text.replace('⌚', '').replace('min read', '')
            
            if len(text) > 100:
                return {
                    "url": url,
                    "text": text,
                    "title": page_title,
                    "status": "success"
                }
        
        return {"url": url, "text": None, "title": None, "status": "failed"}
        
    except Exception as e:
        return {"url": url, "text": None, "title": None, "status": "error", "error": str(e)}
    finally:
        driver.quit()

def chunk_text(text, max_chars=1000, overlap=100):
    """Chunk text with overlap"""
    chunks = []
    
    if not text or len(text) < 50:
        return chunks
    
    start = 0
    while start < len(text):
        end = start + max_chars
        
        if end >= len(text):
            chunk = text[start:].strip()
            if len(chunk) > 30:
                chunks.append(chunk)
            break
        
        chunk_text = text[start:end]
        split_pos = chunk_text.rfind(". ")
        
        if split_pos == -1 or split_pos < max_chars * 0.3:
            split_pos = chunk_text.rfind("! ")
        if split_pos == -1 or split_pos < max_chars * 0.3:
            split_pos = chunk_text.rfind("? ")
        if split_pos == -1 or split_pos < max_chars * 0.3:
            split_pos = chunk_text.rfind("\n")
        if split_pos == -1 or split_pos < max_chars * 0.3:
            split_pos = max_chars
        
        chunk = text[start:start + split_pos].strip()
        
        if chunk and len(chunk) > 30:
            chunks.append(chunk)
            print(f"      📝 Chunk {len(chunks)}: {len(chunk)} chars")
        
        start = start + split_pos - overlap
        
        if len(chunks) >= 100:
            print("      ⚠ Max chunks reached (100)")
            break
    
    return chunks

def generate_chunk_id(text, url):
    """Generate deterministic ID"""
    content = f"{url}:{text[:100]}"
    hash_obj = hashlib.md5(content.encode())
    return int.from_bytes(hash_obj.digest()[:8], byteorder='big')

def embed_batch(texts, retries=3):
    """Embed with retry"""
    for attempt in range(1, retries + 1):
        try:
            print(f"      🧠 Embedding {len(texts)} chunks (attempt {attempt})")
            response = cohere_client.embed(
                model=EMBED_MODEL,
                input_type="search_document",
                texts=texts,
            )
            print("      ✅ Cohere responded")
            return response.embeddings
        except Exception as e:
            print(f"      ⚠ Embed failed: {e}")
            if attempt < retries:
                time.sleep(2 * attempt)
    
    print("      ❌ Giving up on this batch")
    return []

def create_collection():
    """Create or verify collection"""
    print("\n📦 Setting up Qdrant...")
    
    if qdrant_client.collection_exists(COLLECTION_NAME):
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        existing_dim = collection_info.config.params.vectors.size
        
        if existing_dim != VECTOR_SIZE:
            print(f"⚠ Collection exists with WRONG dimensions ({existing_dim} vs {VECTOR_SIZE})")
            print("❌ Please delete the collection manually")
            raise ValueError(f"Dimension mismatch: {existing_dim} != {VECTOR_SIZE}")
        else:
            point_count = collection_info.points_count
            print(f"✅ Collection exists with {point_count} existing points")
            print(f"   Dimension: {VECTOR_SIZE} ✓")
            
            response = input("\n⚠  Delete all data and start fresh? (yes/no): ")
            if response.lower() == 'yes':
                print("🗑  Deleting collection...")
                qdrant_client.delete_collection(COLLECTION_NAME)
                print("📦 Creating fresh collection...")
                qdrant_client.create_collection(
                    collection_name=COLLECTION_NAME,
                    vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
                )
                print("✅ Fresh collection created!")
    else:
        print(f"📦 Creating new collection with {VECTOR_SIZE} dimensions...")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
        )
        print("✅ Collection created")

def save_chunks_batch(chunks_data):
    """Save chunks to Qdrant"""
    texts = [item['chunk'] for item in chunks_data]
    
    print(f"   🔄 Embedding {len(texts)} chunks...")
    vectors = embed_batch(texts)
    
    if not vectors:
        print("   ⚠ Skipping batch due to embed failure")
        return
    
    points = []
    for i, item in enumerate(chunks_data):
        points.append(PointStruct(
            id=item['id'],
            vector=vectors[i],
            payload={
                "url": item['url'],
                "text": item['chunk'],
                "page_title": item['title'],
                "chunk_index": item['chunk_index']
            }
        ))
    
    print(f"   💾 Saving {len(points)} vectors to Qdrant")
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    
    time.sleep(1.2)

def process_and_save_page(result):
    """Process one page"""
    if result['status'] != 'success' or not result['text']:
        return 0
    
    page_name = result['url'].split('/')[-1]
    print(f"\n📄 Processing: {page_name}")
    print(f"   📌 Title: {result['title']}")
    
    chunks = chunk_text(result['text'])
    print(f"   📦 Created {len(chunks)} chunks")
    
    chunks_data = []
    for i, chunk in enumerate(chunks):
        chunk_id = generate_chunk_id(chunk, result['url'])
        chunks_data.append({
            'id': chunk_id,
            'chunk': chunk,
            'url': result['url'],
            'title': result['title'],
            'chunk_index': i
        })
    
    batch_size = 5
    for i in range(0, len(chunks_data), batch_size):
        batch = chunks_data[i:i+batch_size]
        save_chunks_batch(batch)
    
    print(f"   ✅ Saved {len(chunks)} chunks")
    return len(chunks)

def ingest_book():
    """Main ingestion pipeline"""
    create_collection()
    
    print(f"\n🚀 Starting parallel scraping ({len(URLS_TO_SCRAPE)} pages)...")
    print("⏳ This will take ~2-3 minutes...\n")
    
    results = []
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = {executor.submit(extract_text_from_url, url): url for url in URLS_TO_SCRAPE}
        
        for i, future in enumerate(as_completed(futures), 1):
            result = future.result()
            results.append(result)
            
            if result['status'] == 'success':
                print(f"✅ [{i}/{len(URLS_TO_SCRAPE)}] {result['url'].split('/')[-1]} - {len(result['text'])} chars")
            else:
                print(f"❌ [{i}/{len(URLS_TO_SCRAPE)}] {result['url'].split('/')[-1]} - {result['status']}")
    
    print(f"\n💾 Processing and saving pages...")
    
    total_chunks = 0
    successful_pages = 0
    
    for result in results:
        chunks_saved = process_and_save_page(result)
        if chunks_saved > 0:
            total_chunks += chunks_saved
            successful_pages += 1
    
    collection_info = qdrant_client.get_collection(COLLECTION_NAME)
    final_count = collection_info.points_count
    
    print(f"\n" + "="*60)
    print(f"✅ INGESTION COMPLETE!")
    print(f"="*60)
    print(f"📊 Pages processed: {successful_pages}/{len(URLS_TO_SCRAPE)}")
    print(f"📦 Total chunks saved: {total_chunks}")
    print(f"🎯 Total points in collection: {final_count}")
    print(f"="*60)

if __name__ == "__main__":
    ingest_book()
````

## Validation Criteria
- ✅ Scrapes all specified URLs
- ✅ Creates/verifies Qdrant collection
- ✅ Chunks text with proper overlap
- ✅ Generates embeddings via Cohere
- ✅ Uploads vectors to Qdrant successfully

CONSTITUTION 4: RAG Testing Tool
File: rag-testing-tool.md
markdown# RAG Testing Tool Constitution

## Objective
Create a comprehensive testing tool for debugging and validating RAG retrieval quality.

## File: `backend/retrieving.py`

### Complete Implementation
````python
import os
import sys
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_humanoids_robotics")

# Client initialization
print("🔄 Initializing Cohere client...")
try:
    cohere_client = cohere.Client(COHERE_API_KEY)
    print("✅ Cohere client initialized successfully\n")
except Exception as e:
    print(f"❌ Failed to initialize Cohere: {e}")
    sys.exit(1)

print("🔄 Connecting to Qdrant...")
try:
    qdrant = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=120,
        prefer_grpc=False,
        https=True,
        check_compatibility=False,
    )
    
    collections = qdrant.get_collections()
    print("✅ Qdrant connection successful!")
    
    collection_names = [c.name for c in collections.collections]
    print(f"📦 Available collections: {collection_names}\n")
    
    if COLLECTION_NAME not in collection_names:
        print(f"⚠️  Warning: Collection '{COLLECTION_NAME}' not found!")
        sys.exit(1)
        
except Exception as e:
    print(f"❌ Failed to connect to Qdrant: {e}")
    sys.exit(1)

def get_embedding(text):
    """Get embedding vector from Cohere"""
    try:
        response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",
            texts=[text],
        )
        return response.embeddings[0]
    except Exception as e:
        print(f"❌ Embedding generation failed: {e}")
        return None

def retrieve(query, limit=5, score_threshold=0.4):
    """Retrieve relevant chunks"""
    print(f"\n{'='*70}")
    print(f"🔍 QUERY: {query}")
    print(f"{'='*70}")
    print(f"📊 Parameters: limit={limit}, threshold={score_threshold}\n")
    
    embedding = get_embedding(query)
    if embedding is None:
        return []
    
    try:
        result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=limit,
            score_threshold=score_threshold
        )
    except Exception as e:
        print(f"❌ Qdrant query failed: {e}")
        return []
    
    if not result.points:
        print("❌ No results found!\n")
        return []
    
    print(f"✅ Found {len(result.points)} relevant chunks:\n")
    
    chunks = []
    for i, point in enumerate(result.points, 1):
        score = point.score
        text = point.payload.get("text", "")
        url = point.payload.get("url", "N/A")
        title = point.payload.get("page_title", "Unknown")
        
        score_emoji = "🟢" if score >= 0.7 else "🟡" if score >= 0.5 else "🟠"
        
        print(f"{score_emoji} [{i}] Score: {score:.3f}")
        print(f"    📄 Page: {title}")
        print(f"    🔗 URL: {url.split('/')[-1] if '/' in url else url}")
        print(f"    📝 Preview: {text[:150]}...")
        print()
        
        chunks.append({
            "text": text,
            "score": score,
            "url": url,
            "title": title
        })
    
    return chunks

def get_collection_stats():
    """Display collection statistics"""
    try:
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print("\n" + "="*70)
        print("📊 COLLECTION STATISTICS")
        print("="*70)
        print(f"📦 Total chunks: {collection_info.points_count}")
        print(f"🧠 Vector dimensions: {collection_info.config.params.vectors.size}")
        print(f"📏 Distance metric: {collection_info.config.params.vectors.distance}")
        print("="*70 + "\n")
        return True
    except Exception as e:
        print(f"❌ Error getting collection stats: {e}")
        return False

def debug_search(query, limit=20, score_threshold=0.2):
    """Debug mode - show more results with lower threshold"""
    print(f"\n{'='*70}")
    print(f"🐛 DEBUG MODE: {query}")
    print(f"{'='*70}")
    
    embedding = get_embedding(query)
    if embedding is None:
        return []
    
    try:
        result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=limit,
            score_threshold=score_threshold
        )
    except Exception as e:
        print(f"❌ Debug search failed: {e}")
        return []
    
    if not result.points:
        print("❌ No results found!\n")
        return []
    
    print(f"📊 Found {len(result.points)} results:\n")
    
    for i, point in enumerate(result.points, 1):
        print(f"[{i}] Score: {point.score:.3f} | {point.payload.get('page_title', 'Unknown')}")
    
    print()
    return result.points

def run_all_tests():
    """Run comprehensive test suite"""
    
    print("\n🚀 " + "="*66 + " 🚀")
    print("   COMPREHENSIVE RAG RETRIEVAL TEST SUITE")
    print("🚀 " + "="*66 + " 🚀\n")
    
    if not get_collection_stats():
        return
    
    print("\n📝 TEST 1: Technical Query")
    print("-" * 70)
    retrieve("What are ROS2 nodes and topics?", limit=3)
    
    print("\n📝 TEST 2: Hardware Requirements")
    print("-" * 70)
    retrieve("What hardware do I need?", limit=5)
    
    print("\n📝 TEST 3: Conceptual Query")
    print("-" * 70)
    retrieve("Tell me about humanoid robot navigation", limit=5)
    
    print("\n" + "="*70)
    print("✅ TEST SUITE COMPLETE!")
    print("="*70 + "\n")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "debug":
            query = " ".join(sys.argv[2:]) if len(sys.argv) > 2 else "hardware"
            get_collection_stats()
            debug_search(query)
        else:
            query = " ".join(sys.argv[1:])
            get_collection_stats()
            retrieve(query)
    else:
        run_all_tests()
````

## Usage
````bash
# Run test suite
uv run python retrieving.py

# Custom query
uv run python retrieving.py "what is ROS2?"

# Debug mode
uv run python retrieving.py debug hardware
````

## Validation Criteria
- ✅ Connects to Qdrant successfully
- ✅ Retrieves relevant documents
- ✅ Displays scores and metadata
- ✅ Debug mode works for low-threshold searches

CONSTITUTION 5: OpenAI Agents RAG Agent
File: openai-rag-agent.md
markdown# OpenAI Agents RAG Agent Constitution

## Objective
Create an agent-based RAG system using OpenAI Agents SDK with tool calling for document retrieval.

## File: `backend/agent.py`

### Complete Implementation
````python
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import function_tool
import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

load_dotenv()

# OpenRouter setup
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
provider = AsyncOpenAI(
    api_key=openrouter_api_key,
    base_url="https://openrouter.ai/api/v1"
)

model = OpenAIChatCompletionsModel(
    model="mistralai/devstral-2512:free",
    openai_client=provider
)

# RAG clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_humanoids_robotics")

def get_embedding(text):
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

# Verification
print("="*60)
print("STEP 1: Checking Collection")
print("="*60)

try:
    collection_info = qdrant.get_collection(COLLECTION_NAME)
    print(f"✓ Collection exists")
    print(f"  Total documents: {collection_info.points_count}")
    
    if collection_info.points_count == 0:
        print("\n❌ ERROR: Collection is EMPTY!")
        exit()
        
except Exception as e:
    print(f"❌ ERROR: Cannot access collection: {e}")
    exit()

print("\n" + "="*60)
print("STEP 2: Testing Direct Retrieval")
print("="*60)

query = "what is physical ai?"
embedding = get_embedding(query)

result = qdrant.query_points(
    collection_name=COLLECTION_NAME,
    query=embedding,
    limit=5
)

print(f"\nRetrieved {len(result.points)} documents")

if len(result.points) == 0:
    print("\n❌ ERROR: No documents retrieved!")
    exit()

print("\n" + "="*60)
print("STEP 3: Testing Agent with Tool")
print("="*60)

@function_tool
def retrieve(query: str) -> list[str]:
    """Retrieve relevant documents from Qdrant"""
    print(f"\n🔧 Tool called with query: '{query}'")
    
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )
    
    texts = [point.payload["text"] for point in result.points]
    print(f"🔧 Tool returning {len(texts)} documents")
    return texts

agent = Agent(
    name="Assistant",
    instructions="""
    You are an AI tutor for Physical AI & Humanoid Robotics.
    
    IMPORTANT: You MUST call the retrieve tool to get information before answering.
    
    Steps:
    1. Call retrieve(query) tool with the user's question
    2. Use ONLY the returned content to answer
    3. If no relevant content is returned, say "I don't know"
    """,
    model=model,
    tools=[retrieve]
)

result = Runner.run_sync(
    agent,
    input="what is simulation in physical ai?",
)

print("\n" + "="*60)
print("FINAL OUTPUT:")
print("="*60)
print(result.final_output)
````

## Usage
````bash
uv run python agent.py
````

## Validation Criteria
- ✅ Agent initializes with OpenRouter model
- ✅ Retrieve tool is called automatically
- ✅ Documents are retrieved from Qdrant
- ✅ Agent generates response based on retrieved context

CONSTITUTION 6: Frontend Chatbot Component
File: frontend-chatbot.md
markdown# Frontend Chatbot Component Constitution

## Objective
Create an enhanced React chatbot component with persistent chat history, text selection queries, responsive design, and clear chat functionality.

## File: `src/components/chatbot/HumanoidChatbot.jsx`

### Requirements
1. **Persistent Chat History**: Messages persist across page reloads using localStorage
2. **Text Selection Query**: User can select text on page and query about it
3. **Clear Chat Button**: Manual chat clearing functionality
4. **Enhanced Responsive UI**: Improved design that fits Docusaurus theme
5. **Better UX**: Loading states, error handling, smooth animations

### Complete Implementation
````jsx
import React, { useState, useRef, useEffect } from 'react';
import { MessageCircle, X, Send, Bot, User, Loader2, Trash2, Info } from 'lucide-react';
import './HumanoidChatbot.css';

const HumanoidChatbot = () => {
  // Load chat history from localStorage or use default message
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState(() => {
    const savedMessages = localStorage.getItem('chatHistory');
    if (savedMessages) {
      return JSON.parse(savedMessages);
    }
    return [
      {
        type: 'ai',
        text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation. You can also select any text on the page and click "Ask about this" to query it!',
        timestamp: new Date().toISOString()
      }
    ];
  });
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [showTextPopup, setShowTextPopup] = useState(false);
  const [popupPosition, setPopupPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef(null);

  // Save messages to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('chatHistory', JSON.stringify(messages));
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Text selection handler
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      
      if (text && text.length > 0 && text.length < 500) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        setSelectedText(text);
        setPopupPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 10
        });
        setShowTextPopup(true);
      } else {
        setShowTextPopup(false);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  const handleAskAboutSelection = () => {
    if (selectedText) {
      setInput(`Explain this: "${selectedText}"`);
      setIsOpen(true);
      setShowTextPopup(false);
      window.getSelection()?.removeAllRanges();
    }
  };

  const handleClearChat = () => {
    const confirmClear = window.confirm('Are you sure you want to clear the chat history?');
    if (confirmClear) {
      const welcomeMessage = {
        type: 'ai',
        text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation.',
        timestamp: new Date().toISOString()
      };
      setMessages([welcomeMessage]);
      localStorage.setItem('chatHistory', JSON.stringify([welcomeMessage]));
    }
  };

  const handleSend = async () => {
    if (!input.trim() || isTyping) return;

    const userMessage = {
      type: 'user',
      text: input,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;
    setInput('');
    setIsTyping(true);

    try {
      const response = await fetch("http://localhost:8000/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          message: currentInput,
          conversation_history: messages.slice(-10)
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const aiMessage = {
        type: 'ai',
        text: data.response || 'I apologize, but I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage = {
        type: 'ai',
        text: '⚠️ Sorry, I am currently unable to process your request. Please make sure the backend server is running on http://localhost:8000',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <>
      {/* Text Selection Popup */}
      {showTextPopup && (
        <div
          className="text-selection-popup"
          style={{
            left: `${popupPosition.x}px`,
            top: `${popupPosition.y}px`,
          }}
        >
          <button
            onClick={handleAskAboutSelection}
            className="ask-about-button"
          >
            <Info size={14} />
            Ask about this
          </button>
        </div>
      )}

      {/* Main Chatbot Container */}
      <div className="chatbot-container">
        {/* Chatbot Window */}
        <div className={`chatbot-window ${isOpen ? 'chatbot-open' : ''}`}>
          {/* Header */}
          <div className="chatbot-header">
            <div className="header-content">
              <div className="bot-avatar-container">
                <div className="bot-avatar">
                  <Bot className="bot-icon" />
                </div>
                <div className="status-indicator"></div>
              </div>
              <div className="header-text">
                <h3 className="bot-title">Robotics AI Tutor</h3>
                <p className="bot-subtitle">Powered by Gemini AI + RAG</p>
              </div>
            </div>
            <div className="header-actions">
              <button
                onClick={handleClearChat}
                className="icon-button clear-button"
                title="Clear chat history"
              >
                <Trash2 size={18} />
              </button>
              <button
                onClick={() => setIsOpen(false)}
                className="icon-button close-button"
              >
                <X size={20} />
              </button>
            </div>
          </div>

          {/* Messages Container */}
          <div className="messages-container">
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`message-wrapper ${msg.type === 'user' ? 'user-message' : 'ai-message'}`}
              >
                {/* Avatar */}
                <div className={`message-avatar ${msg.type}-avatar`}>
                  {msg.type === 'ai' ? (
                    <Bot size={20} />
                  ) : (
                    <User size={20} />
                  )}
                </div>

                {/* Message Bubble */}
                <div className="message-content">
                  <div className={`message-bubble ${msg.type}-bubble`}>
                    {msg.type === 'ai' && (
                      <div className="ai-badge">
                        <div className="pulse-dot"></div>
                        <span>Gemini AI (RAG-Enhanced)</span>
                      </div>
                    )}
                    <p className="message-text">{msg.text}</p>
                  </div>
                  <span className="message-timestamp">
                    {new Date(msg.timestamp).toLocaleTimeString([], { 
                      hour: '2-digit', 
                      minute: '2-digit' 
                    })}
                  </span>
                </div>
              </div>
            ))}

            {isTyping && (
              <div className="message-wrapper ai-message">
                <div className="message-avatar ai-avatar">
                  <Loader2 size={20} className="spinning" />
                </div>
                <div className="message-content">
                  <div className="message-bubble ai-bubble typing-indicator">
                    <div className="typing-dots">
                      <div className="dot"></div>
                      <div className="dot"></div>
                      <div className="dot"></div>
                    </div>
                    <span className="typing-text">Searching knowledge base...</span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="input-container">
            <div className="input-wrapper">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about robotics, AI, simulation..."
                disabled={isTyping}
                className="chat-input"
              />
              <button
                onClick={handleSend}
                disabled={!input.trim() || isTyping}
                className="send-button"
              >
                {isTyping ? (
                  <Loader2 size={20} className="spinning" />
                ) : (
                  <Send size={20} />
                )}
              </button>
            </div>
            <p className="powered-by">
              Powered by Qdrant + Cohere + Gemini AI
            </p>
          </div>
        </div>

        {/* Floating Button */}
        <button
          onClick={() => setIsOpen(!isOpen)}
          className="floating-button"
        >
          {isOpen ? (
            <X size={28} />
          ) : (
            <>
              <MessageCircle size={28} />
              <div className="notification-badge"></div>
            </>
          )}
        </button>
      </div>
    </>
  );
};

export default HumanoidChatbot;
````

---

## File: `src/components/chatbot/HumanoidChatbot.css`
````css
/* ============================================
   CHATBOT CONTAINER & LAYOUT
   ============================================ */

.chatbot-container {
  position: fixed;
  bottom: 24px;
  right: 24px;
  z-index: 9999;
  font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif;
}

/* ============================================
   CHATBOT WINDOW
   ============================================ */

.chatbot-window {
  position: absolute;
  bottom: 80px;
  right: 0;
  width: 400px;
  max-width: calc(100vw - 48px);
  height: 600px;
  max-height: calc(100vh - 120px);
  background: var(--ifm-background-color, white);
  border-radius: 16px;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.15);
  border: 1px solid var(--ifm-color-emphasis-200);
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  transform: scale(0.95) translateY(20px);
  opacity: 0;
  pointer-events: none;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.chatbot-window.chatbot-open {
  transform: scale(1) translateY(0);
  opacity: 1;
  pointer-events: all;
}

/* ============================================
   HEADER
   ============================================ */

.chatbot-header {
  background: linear-gradient(135deg, #16a34a 0%, #14b8a6 100%);
  padding: 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  border-radius: 16px 16px 0 0;
  color: white;
}

.header-content {
  display: flex;
  align-items: center;
  gap: 12px;
  flex: 1;
}

.bot-avatar-container {
  position: relative;
}

.bot-avatar {
  width: 40px;
  height: 40px;
  background: white;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

.bot-icon {
  color: #16a34a;
  width: 24px;
  height: 24px;
}

.status-indicator {
  position: absolute;
  bottom: -2px;
  right: -2px;
  width: 12px;
  height: 12px;
  background: #22c55e;
  border-radius: 50%;
  border: 2px solid white;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% {
    opacity: 1;
  }
  50% {
    opacity: 0.5;
  }
}

.header-text {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.bot-title {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  line-height: 1.2;
}

.bot-subtitle {
  margin: 0;
  font-size: 11px;
  opacity: 0.9;
  line-height: 1.2;
}

.header-actions {
  display: flex;
  gap: 8px;
}

.icon-button {
  background: rgba(255, 255, 255, 0.2);
  border: none;
  color: white;
  width: 32px;
  height: 32px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.2s;
}

.icon-button:hover {
  background: rgba(255, 255, 255, 0.3);
  transform: scale(1.05);
}

.icon-button:active {
  transform: scale(0.95);
}

/* ============================================
   MESSAGES CONTAINER
   ============================================ */

.messages-container {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  background: linear-gradient(to bottom, 
    var(--ifm-background-color) 0%,
    var(--ifm-background-surface-color, #f9fafb) 100%
  );
  display: flex;
  flex-direction: column;
  gap: 16px;
}

/* Custom Scrollbar */
.messages-container::-webkit-scrollbar {
  width: 6px;
}

.messages-container::-webkit-scrollbar-track {
  background: rgba(21, 128, 61, 0.05);
  border-radius: 10px;
}

.messages-container::-webkit-scrollbar-thumb {
  background: linear-gradient(to bottom, #16a34a, #14b8a6);
  border-radius: 10px;
}

.messages-container::-webkit-scrollbar-thumb:hover {
  background: linear-gradient(to bottom, #15803d, #0d9488);
}

/* ============================================
   MESSAGES
   ============================================ */

.message-wrapper {
  display: flex;
  gap: 12px;
  animation: messageSlide 0.3s ease-out;
}

@keyframes messageSlide {
  from {
    opacity: 0;
    transform: translateY(10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.user-message {
  flex-direction: row-reverse;
}

.message-avatar {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  color: white;
}

.ai-avatar {
  background: linear-gradient(135deg, #16a34a 0%, #14b8a6 100%);
}

.user-avatar {
  background: linear-gradient(135deg, #3b82f6 0%, #6366f1 100%);
}

.message-content {
  display: flex;
  flex-direction: column;
  gap: 4px;
  max-width: 75%;
}

.user-message .message-content {
  align-items: flex-end;
}

.message-bubble {
  padding: 12px 16px;
  border-radius: 16px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
}

.ai-bubble {
  background: var(--ifm-background-color, white);
  border: 1px solid var(--ifm-color-emphasis-200);
  color: var(--ifm-font-color-base);
}

.user-bubble {
  background: linear-gradient(135deg, #3b82f6 0%, #6366f1 100%);
  color: white;
}

.ai-badge {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 11px;
  font-weight: 600;
  color: #16a34a;
  margin-bottom: 8px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--ifm-color-emphasis-200);
}

.pulse-dot {
  width: 6px;
  height: 6px;
  background: #22c55e;
  border-radius: 50%;
  animation: pulse 2s infinite;
}

.message-text {
  margin: 0;
  font-size: 14px;
  line-height: 1.6;
  white-space: pre-wrap;
  word-wrap: break-word;
}

.message-timestamp {
  font-size: 11px;
  color: var(--ifm-color-emphasis-600);
  padding: 0 8px;
}

/* ============================================
   TYPING INDICATOR
   ============================================ */

.typing-indicator {
  display: flex;
  align-items: center;
  gap: 12px;
}

.typing-dots {
  display: flex;
  gap: 4px;
}

.dot {
  width: 6px;
  height: 6px;
  background: #16a34a;
  border-radius: 50%;
  animation: bounce 1.4s infinite;
}

.dot:nth-child(2) {
  animation-delay: 0.2s;
}

.dot:nth-child(3) {
  animation-delay: 0.4s;
}

@keyframes bounce {
  0%, 60%, 100% {
    transform: translateY(0);
  }
  30% {
    transform: translateY(-8px);
  }
}

.typing-text {
  font-size: 12px;
  color: var(--ifm-color-emphasis-600);
}

/* ============================================
   INPUT AREA
   ============================================ */

.input-container {
  padding: 16px;
  background: var(--ifm-background-color, white);
  border-top: 1px solid var(--ifm-color-emphasis-200);
  border-radius: 0 0 16px 16px;
}

.input-wrapper {
  display: flex;
  gap: 8px;
  margin-bottom: 8px;
}

.chat-input {
  flex: 1;
  padding: 12px 16px;
  background: var(--ifm-background-surface-color, #f9fafb);
  border: 1px solid var(--ifm-color-emphasis-200);
  border-radius: 12px;
  font-size: 14px;
  color: var(--ifm-font-color-base);
  transition: all 0.2s;
  outline: none;
}

.chat-input:focus {
  border-color: #16a34a;
  box-shadow: 0 0 0 3px rgba(22, 163, 74, 0.1);
}

.chat-input:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.send-button {
  width: 44px;
  height: 44px;
  background: linear-gradient(135deg, #16a34a 0%, #14b8a6 100%);
  border: none;
  border-radius: 12px;
  color: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s;
  box-shadow: 0 4px 12px rgba(22, 163, 74, 0.3);
}

.send-button:hover:not(:disabled) {
  transform: scale(1.05);
  box-shadow: 0 6px 16px rgba(22, 163, 74, 0.4);
}

.send-button:active:not(:disabled) {
  transform: scale(0.95);
}

.send-button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.powered-by {
  margin: 0;
  font-size: 11px;
  color: var(--ifm-color-emphasis-600);
  text-align: center;
}

/* ============================================
   FLOATING BUTTON
   ============================================ */

.floating-button {
  width: 64px;
  height: 64px;
  background: linear-gradient(135deg, #16a34a 0%, #14b8a6 100%);
  border: none;
  border-radius: 50%;
  color: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 8px 24px rgba(22, 163, 74, 0.4);
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
  animation: float 3s ease-in-out infinite;
}

@keyframes float {
  0%, 100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-10px);
  }
}

.floating-button:hover {
  transform: scale(1.1);
  box-shadow: 0 12px 32px rgba(22, 163, 74, 0.5);
}

.floating-button:active {
  transform: scale(0.95);
}

.notification-badge {
  position: absolute;
  top: -4px;
  right: -4px;
  width: 16px;
  height: 16px;
  background: #ef4444;
  border-radius: 50%;
  border: 2px solid white;
  animation: pulse 2s infinite;
}

/* ============================================
   TEXT SELECTION POPUP
   ============================================ */

.text-selection-popup {
  position: fixed;
  transform: translate(-50%, -100%);
  z-index: 10000;
  animation: popupSlide 0.2s ease-out;
}

@keyframes popupSlide {
  from {
    opacity: 0;
    transform: translate(-50%, -90%);
  }
  to {
    opacity: 1;
    transform: translate(-50%, -100%);
  }
}

.ask-about-button {
  background: linear-gradient(135deg, #16a34a 0%, #14b8a6 100%);
  color: white;
  border: none;
  padding: 8px 16px;
  border-radius: 8px;
  font-size: 13px;
  font-weight: 500;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 6px;
  box-shadow: 0 4px 12px rgba(22, 163, 74, 0.3);
  transition: all 0.2s;
  white-space: nowrap;
}

.ask-about-button:hover {
  transform: scale(1.05);
  box-shadow: 0 6px 16px rgba(22, 163, 74, 0.4);
}

.ask-about-button:active {
  transform: scale(0.95);
}

/* ============================================
   UTILITY CLASSES
   ============================================ */

.spinning {
  animation: spin 1s linear infinite;
}

@keyframes spin {
  from {
    transform: rotate(0deg);
  }
  to {
    transform: rotate(360deg);
  }
}

/* ============================================
   RESPONSIVE DESIGN
   ============================================ */

@media (max-width: 768px) {
  .chatbot-window {
    width: calc(100vw - 32px);
    height: calc(100vh - 100px);
    bottom: 70px;
    right: 16px;
  }

  .chatbot-container {
    bottom: 16px;
    right: 16px;
  }

  .floating-button {
    width: 56px;
    height: 56px;
  }

  .message-content {
    max-width: 85%;
  }
}

@media (max-width: 480px) {
  .bot-title {
    font-size: 14px;
  }

  .bot-subtitle {
    font-size: 10px;
  }

  .message-text {
    font-size: 13px;
  }

  .chat-input {
    font-size: 13px;
  }
}

/* ============================================
   DARK MODE SUPPORT
   ============================================ */

[data-theme='dark'] .chatbot-window {
  background: var(--ifm-background-color);
  border-color: var(--ifm-color-emphasis-300);
}

[data-theme='dark'] .messages-container {
  background: linear-gradient(to bottom,
    var(--ifm-background-color) 0%,
    var(--ifm-background-surface-color) 100%
  );
}

[data-theme='dark'] .ai-bubble {
  background: var(--ifm-background-surface-color);
  border-color: var(--ifm-color-emphasis-300);
}

[data-theme='dark'] .chat-input {
  background: var(--ifm-background-surface-color);
  border-color: var(--ifm-color-emphasis-300);
  color: var(--ifm-font-color-base);
}

[data-theme='dark'] .input-container {
  background: var(--ifm-background-color);
  border-color: var(--ifm-color-emphasis-300);
}
````

## Integration Instructions

### Step 1: Import in Root/Layout
File: `src/theme/Root.js` (create if doesn't exist)
````jsx
import React from 'react';
import HumanoidChatbot from '@site/src/components/chatbot/HumanoidChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <HumanoidChatbot />
    </>
  );
}
````

### Step 2: Ensure Backend is Running
````bash
cd backend
uv run python main.py
````

## Validation Criteria
- ✅ Chat history persists across page reloads
- ✅ Text selection popup appears when selecting text
- ✅ "Ask about this" button works correctly
- ✅ Clear chat button clears history with confirmation
- ✅ Responsive design works on mobile/tablet/desktop
- ✅ Dark mode support works properly
- ✅ Smooth animations and transitions
- ✅ Backend connection works
- ✅ Error handling displays properly

CONSTITUTION 7: Integration & Deployment Guide
File: integration-deployment.md
markdown# Integration & Deployment Guide

## Complete Setup Process

### 1. Backend Setup
````bash
# Navigate to backend directory
cd backend

# Initialize UV project (if not done)
uv init

# Install dependencies
uv sync

# Create .env file
cp .env.example .env
# Edit .env with your API keys

# Run scraper to ingest documentation (FIRST TIME ONLY)
uv run python scrapper.py

# Start FastAPI server
uv run python main.py
````

Server will run on: `http://localhost:8000`

### 2. Frontend Integration

#### Option A: Using Theme Root (Recommended)

Create `src/theme/Root.js`:
````jsx
import React from 'react';
import HumanoidChatbot from '@site/src/components/chatbot/HumanoidChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <HumanoidChatbot />
    </>
  );
}
````

#### Option B: Direct Import in docusaurus.config.js
````js
module.exports = {
  // ... other config
  clientModules: [
    require.resolve('./src/components/chatbot/HumanoidChatbot.jsx'),
  ],
};
````

### 3. Run Docusaurus
````bash
# Development
npm start

# Production build
npm run build
npm run serve
````

### 4. Testing Checklist

- [ ] Backend health check: `http://localhost:8000/health`
- [ ] API docs: `http://localhost:8000/docs`
- [ ] Chatbot appears on all pages
- [ ] Chat history persists after page reload
- [ ] Text selection "Ask about this" works
- [ ] Clear chat button works
- [ ] Messages send and receive properly
- [ ] Responsive on mobile/tablet/desktop
- [ ] Dark mode works correctly

### 5. Production Deployment

#### Backend (Railway/Render/Fly.io)

**Environment Variables to Set:**
````env
COHERE_API_KEY=xxx
QDRANT_URL=xxx
QDRANT_API_KEY=xxx
COLLECTION_NAME=physical_ai_humanoids_robotics
OPENROUTER_API_KEY=xxx
````

**Update CORS in main.py:**
````python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://your-domain.com"],  # Your production domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
````

#### Frontend Update

Update API URL in `HumanoidChatbot.jsx`:
````jsx
const response = await fetch("https://your-backend.com/chat", {
  // ...
});
````

### 6. Monitoring & Maintenance

**Check Collection Stats:**
````bash
uv run python retrieving.py
````

**Update Documentation:**
````bash
# Re-run scraper to update Qdrant with new content
uv run python scrapper.py
````

**Test RAG Quality:**
````bash
# Test specific queries
uv run python retrieving.py "your query here"

# Debug mode
uv run python retrieving.py debug "hardware"
````

## Troubleshooting

### Backend Won't Start
- Check `.env` file has all required keys
- Verify Qdrant cluster is active
- Test Cohere API key validity

### No Search Results
- Run `uv run python scrapper.py` to ingest docs
- Check collection has data: visit `/collection-stats`
- Test with `retrieving.py`

### Frontend Connection Error
- Ensure backend is running on `http://localhost:8000`
- Check CORS settings in `main.py`
- Verify network requests in browser DevTools

### Chat History Not Persisting
- Check browser localStorage is enabled
- Clear cache and reload
- Verify `localStorage.setItem` is working

## Validation Criteria

Final checklist before submission:

- ✅ Backend runs without errors
- ✅ Qdrant has ingested documentation
- ✅ Frontend chatbot renders properly
- ✅ All 4 new features work:
  - Persistent chat history
  - Text selection query
  - Clear chat button
  - Enhanced responsive UI
- ✅ RAG retrieval returns relevant results
- ✅ Dark mode works
- ✅ Mobile responsive
- ✅ Production-ready error handling

Summary of Constitutions

backend-setup.md - UV project initialization, dependencies, environment setup
fastapi-server.md - Complete FastAPI server with RAG endpoints
web-scraper.md - Selenium-based documentation scraper
rag-testing-tool.md - Retrieval testing and debugging tool
openai-rag-agent.md - Agent-based RAG implementation
frontend-chatbot.md - Enhanced React chatbot component with new features
integration-deployment.md - Complete setup and deployment guide

Each constitution is independent and can be executed separately by Claude Code CLI without overriding others. All constitutions are stored in a dedicated folder structure.