"""
FastAPI Backend for Humanoid Robotics Chatbot
Integrates with Cohere, Qdrant, and OpenRouter AI
Railway-Optimized Version
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict
import os
from datetime import datetime
from dotenv import load_dotenv  

load_dotenv()

# ============================================================================
# CONFIGURATION
# ============================================================================

# Cohere & Qdrant Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY", "")
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
COLLECTION_NAME = "physical_ai_humanoids_robotics"
EMBED_MODEL = "embed-english-v3.0"

# OpenRouter Configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "")
OPENROUTER_MODEL = "mistralai/devstral-2512:free"

# Initialize clients with error handling
cohere_client = None
qdrant_client = None
openrouter_client = None

try:
    import cohere
    if COHERE_API_KEY:
        cohere_client = cohere.Client(COHERE_API_KEY)
        print("✅ Cohere initialized")
    else:
        print("⚠️ Cohere API key missing - RAG disabled")
except Exception as e:
    print(f"⚠️ Cohere initialization failed: {e}")

try:
    from qdrant_client import QdrantClient
    if QDRANT_URL and QDRANT_API_KEY:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print("✅ Qdrant initialized")
    else:
        print("⚠️ Qdrant credentials missing - RAG disabled")
except Exception as e:
    print(f"⚠️ Qdrant initialization failed: {e}")

try:
    from openai import OpenAI
    if OPENROUTER_API_KEY:
        openrouter_client = OpenAI(
            api_key=OPENROUTER_API_KEY,
            base_url="https://openrouter.ai/api/v1"
        )
        print("✅ OpenRouter initialized")
    else:
        print("⚠️ OpenRouter API key missing - AI disabled")
except Exception as e:
    print(f"⚠️ OpenRouter initialization failed: {e}")

# In-memory conversation storage
conversations: Dict[str, List[Dict]] = {}

# ============================================================================
# FASTAPI APP
# ============================================================================

app = FastAPI(
    title="Humanoid Robotics Chatbot API",
    description="RAG-powered chatbot using Cohere, Qdrant, and OpenRouter AI",
    version="1.0.0"
)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# PYDANTIC MODELS
# ============================================================================

class ChatRequest(BaseModel):
    message: str
    conversation_id: str
    selected_text: Optional[str] = None

class Source(BaseModel):
    url: str
    title: str
    snippet: str
    score: float

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    timestamp: str
    conversation_id: str

class ClearRequest(BaseModel):
    conversation_id: str

# ============================================================================
# RAG FUNCTIONS
# ============================================================================

def search_knowledge_base(query: str, top_k: int = 5) -> List[Dict]:
    """
    Search Qdrant vector database for relevant documents
    """
    if not cohere_client or not qdrant_client:
        print("⚠️ RAG services not available")
        return []
    
    try:
        print(f"🔍 Searching for: '{query}'")
        
        # Embed the query using Cohere
        query_embedding = cohere_client.embed(
            model=EMBED_MODEL,
            input_type="search_query",
            texts=[query]
        ).embeddings[0]
        
        print(f"✅ Query embedded successfully")
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k
        )
        
        print(f"✅ Found {len(search_results)} results from Qdrant")
        
        # Format results
        results = []
        for hit in search_results:
            results.append({
                "text": hit.payload.get("text", ""),
                "url": hit.payload.get("url", ""),
                "title": hit.payload.get("page_title", "Unknown"),
                "score": hit.score,
                "chunk_index": hit.payload.get("chunk_index", 0)
            })
            print(f"   📄 {hit.payload.get('page_title', 'Unknown')} (score: {hit.score:.3f})")
        
        return results
    
    except Exception as e:
        print(f"❌ Search error: {e}")
        return []

def generate_response(query: str, context_docs: List[Dict], conversation_history: List[Dict], selected_text: Optional[str] = None) -> str:
    """
    Generate response using OpenRouter AI
    """
    if not openrouter_client:
        print("⚠️ AI service not available - using fallback")
        return generate_fallback_response(query)
    
    try:
        print("🤖 Generating response with OpenRouter (Mistral)...")
        
        # Build context from retrieved documents
        context = "\n\n".join([
            f"Source: {doc['title']}\nContent: {doc['text'][:600]}..."
            for doc in context_docs[:3]
        ])
        
        # Build conversation history (last 5 messages)
        history_messages = []
        for msg in conversation_history[-5:]:
            history_messages.append({
                "role": msg["role"],
                "content": msg["content"]
            })
        
        # Construct system prompt
        system_prompt = """You are an expert AI tutor specializing in Humanoid Robotics and Physical AI.

Your role:
- Provide accurate, helpful answers about robotics, AI, ROS2, simulation, and related topics
- Use the provided documentation context to give precise answers
- Be concise but thorough (2-4 paragraphs maximum)
- If you're unsure, acknowledge it honestly
- Include practical examples when relevant
- Use technical terminology appropriately but explain complex concepts clearly"""

        # Construct user prompt
        if selected_text:
            user_prompt = f"""USER SELECTED TEXT FROM DOCUMENTATION:
"{selected_text}"

USER QUESTION:
{query}

RELEVANT DOCUMENTATION:
{context}

Please answer the question, focusing on the selected text and using the documentation context."""
        else:
            user_prompt = f"""USER QUESTION:
{query}

RELEVANT DOCUMENTATION:
{context}

Please answer the question using the documentation context provided."""
        
        # Prepare messages for OpenRouter
        messages = [
            {"role": "system", "content": system_prompt}
        ]
        
        # Add conversation history (if exists)
        if history_messages:
            messages.extend(history_messages[:-1])
        
        # Add current question
        messages.append({"role": "user", "content": user_prompt})
        
        # Call OpenRouter API
        response = openrouter_client.chat.completions.create(
            model=OPENROUTER_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=800,
            extra_headers={
                "HTTP-Referer": "https://physical-ai-and-humanoids-robotics.vercel.app",
                "X-Title": "Humanoid Robotics Chatbot"
            }
        )
        
        result = response.choices[0].message.content
        print(f"✅ Generated {len(result)} characters")
        return result
    
    except Exception as e:
        print(f"❌ OpenRouter generation error: {e}")
        return generate_fallback_response(query)

def generate_fallback_response(query: str) -> str:
    """
    Generate simple fallback response when AI is unavailable
    """
    query_lower = query.lower()
    
    if any(word in query_lower for word in ["hello", "hi", "hey"]):
        return "Hello! I'm the Humanoid Robotics assistant. I'm currently operating in limited mode. Please check our documentation for detailed information on robotics and AI topics."
    
    elif any(word in query_lower for word in ["ros", "ros2", "robot"]):
        return "I'd love to help with ROS2 and robotics questions! I'm currently in maintenance mode with limited AI capabilities. Please refer to our comprehensive documentation for detailed guides on ROS2, robotics simulation, and humanoid systems."
    
    elif any(word in query_lower for word in ["python", "code", "programming"]):
        return "For programming and coding questions, I recommend checking our documentation which includes practical examples and tutorials on robotics development, ROS2 programming, and AI integration."
    
    else:
        return "Thank you for your question! I'm currently operating with limited capabilities. Please explore our documentation at https://physical-ai-and-humanoids-robotics.vercel.app for comprehensive information on humanoid robotics, physical AI, and related topics."

# ============================================================================
# API ENDPOINTS
# ============================================================================

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Humanoid Robotics Chatbot API",
        "version": "1.0.0",
        "message": "Backend is running!"
    }

@app.get("/health")
async def health():
    """Simple health check"""
    return {"status": "ok"}

@app.post("/api/v1/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint - handles user queries with RAG
    """
    try:
        print("\n" + "="*60)
        print(f"💬 New chat request: {request.message[:50]}...")
        print("="*60)
        
        # Get or create conversation history
        if request.conversation_id not in conversations:
            conversations[request.conversation_id] = []
            print(f"✅ Created new conversation: {request.conversation_id}")
        
        conversation_history = conversations[request.conversation_id]
        
        # Add user message to history
        conversation_history.append({
            "role": "user",
            "content": request.message,
            "timestamp": datetime.now().isoformat()
        })
        
        # Search knowledge base
        print(f"\n🔍 Step 1: Searching knowledge base...")
        search_results = search_knowledge_base(request.message, top_k=5)
        
        if not search_results:
            print("⚠️ No search results found - providing general response")
        
        # Generate response
        print(f"\n🤖 Step 2: Generating AI response...")
        ai_response = generate_response(
            query=request.message,
            context_docs=search_results,
            conversation_history=conversation_history,
            selected_text=request.selected_text
        )
        
        # Add AI response to history
        conversation_history.append({
            "role": "assistant",
            "content": ai_response,
            "timestamp": datetime.now().isoformat()
        })
        
        # Keep only last 20 messages
        if len(conversation_history) > 20:
            conversation_history = conversation_history[-20:]
            conversations[request.conversation_id] = conversation_history
        
        # Format sources
        sources = [
            Source(
                url=result["url"],
                title=result["title"],
                snippet=result["text"][:150] + "...",
                score=result["score"]
            )
            for result in search_results[:3]
        ]
        
        print(f"\n✅ Response generated successfully!")
        print(f"📊 Sources: {len(sources)}")
        print(f"📝 Response length: {len(ai_response)} chars")
        print("="*60 + "\n")
        
        return ChatResponse(
            response=ai_response,
            sources=sources,
            timestamp=datetime.now().isoformat(),
            conversation_id=request.conversation_id
        )
    
    except Exception as e:
        print(f"\n❌ ERROR in chat endpoint: {e}")
        # Return error response instead of raising exception
        return ChatResponse(
            response="I apologize, but I encountered an error. Please try again.",
            sources=[],
            timestamp=datetime.now().isoformat(),
            conversation_id=request.conversation_id
        )

@app.post("/api/v1/chat/clear")
async def clear_history(request: ClearRequest):
    """
    Clear conversation history
    """
    try:
        if request.conversation_id in conversations:
            del conversations[request.conversation_id]
            print(f"🗑️ Cleared conversation: {request.conversation_id}")
            return {"status": "success", "message": "Conversation history cleared"}
        return {"status": "success", "message": "No history found"}
    
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/api/v1/health")
async def health_check():
    """
    Health check - verifies all services
    """
    status = {
        "status": "healthy",
        "cohere": {"connected": cohere_client is not None},
        "qdrant": {"connected": qdrant_client is not None},
        "openrouter": {"connected": openrouter_client is not None, "model": OPENROUTER_MODEL}
    }
    
    if qdrant_client:
        try:
            collection_info = qdrant_client.get_collection(COLLECTION_NAME)
            status["qdrant"]["collection"] = COLLECTION_NAME
            status["qdrant"]["points"] = collection_info.points_count
        except Exception as e:
            status["qdrant"]["error"] = str(e)
    
    return status

# ============================================================================
# RUN SERVER
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    
    # Get port from environment (Railway provides this)
    port = int(os.getenv("PORT", 8000))
    
    print("\n" + "="*70)
    print("🚀 Starting Humanoid Robotics Chatbot API")
    print("="*70)
    print(f"📍 Port: {port}")
    print(f"🤖 AI Model: {OPENROUTER_MODEL}")
    print(f"💾 Cohere: {'✅ Available' if cohere_client else '❌ Unavailable'}")
    print(f"💾 Qdrant: {'✅ Available' if qdrant_client else '❌ Unavailable'}")
    print(f"🤖 OpenRouter: {'✅ Available' if openrouter_client else '❌ Unavailable'}")
    print("="*70 + "\n")
    
    # Run server with production settings
    uvicorn.run(
        app, 
        host="0.0.0.0",  # Must be 0.0.0.0 for Railway
        port=port,
        reload=False  # No auto-reload in production
    )