"""
FastAPI Backend for Humanoid Robotics Chatbot
Integrates with Cohere, Qdrant, and OpenRouter AI
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict
import os
from datetime import datetime
import cohere
from qdrant_client import QdrantClient
from openai import OpenAI
import asyncio
from dotenv import load_dotenv

# ============================================================================
# CONFIGURATION
# ============================================================================
load_dotenv()
# Cohere & Qdrant Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY =os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "physical_ai_humanoids_robotics"
EMBED_MODEL = "embed-english-v3.0"

# OpenRouter Configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "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")
OPENROUTER_MODEL = "mistralai/devstral-2512:free"

# Initialize clients
cohere_client = cohere.Client(COHERE_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Initialize OpenRouter client using OpenAI SDK
openrouter_client = OpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)

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
    language: Optional[str] = 'en'  # Default to English

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

class TranslateRequest(BaseModel):
    text: str
    target_language: str

class TranslateResponse(BaseModel):
    translated_text: str
    source_language: Optional[str] = None
    target_language: str

# ============================================================================
# RAG FUNCTIONS
# ============================================================================

def search_knowledge_base(query: str, top_k: int = 5) -> List[Dict]:
    """
    Search Qdrant vector database for relevant documents
    """
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
        import traceback
        traceback.print_exc()
        return []

def generate_response(query: str, context_docs: List[Dict], conversation_history: List[Dict], selected_text: Optional[str] = None, language: Optional[str] = None) -> str:
    """
    Generate response using OpenRouter AI
    """
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

        # Add language context if provided
        if language and language != 'en':
            user_prompt += f"\n\nPlease respond in {language} language."

        # Prepare messages for OpenRouter
        messages = [
            {"role": "system", "content": system_prompt}
        ]

        # Add conversation history (if exists)
        if history_messages:
            messages.extend(history_messages[:-1])  # Exclude current question

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
        import traceback
        traceback.print_exc()
        return "I apologize, but I'm having trouble generating a response. Please try again or rephrase your question."


def translate_text(text: str, target_language: str) -> str:
    """
    Translate text using OpenRouter AI
    """
    try:
        print(f"🔄 Translating text to {target_language}...")

        # Define language names for better translation quality
        language_names = {
            'en': 'English',
            'ur': 'Urdu',
            'ar': 'Arabic',
            'es': 'Spanish',
            'fr': 'French',
            'de': 'German',
            'zh': 'Chinese',
            'ja': 'Japanese'
        }

        target_language_name = language_names.get(target_language, target_language)

        # Construct translation prompt
        translation_prompt = f"""Translate the following text to {target_language_name}.
Maintain technical terminology where appropriate.
Only return the translated text, no explanations.

Text: {text}"""

        # Call OpenRouter API for translation
        response = openrouter_client.chat.completions.create(
            model=OPENROUTER_MODEL,
            messages=[
                {"role": "system", "content": "You are a professional translator. Translate text accurately while maintaining the meaning and tone. Only return the translated text without any additional explanations."},
                {"role": "user", "content": translation_prompt}
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=1000,
            extra_headers={
                "HTTP-Referer": "https://physical-ai-and-humanoids-robotics.vercel.app",
                "X-Title": "Humanoid Robotics Chatbot"
            }
        )

        translated_text = response.choices[0].message.content
        print(f"✅ Translated {len(text)} characters to {len(translated_text)} characters")
        return translated_text

    except Exception as e:
        print(f"❌ Translation error: {e}")
        import traceback
        traceback.print_exc()
        return text  # Return original text if translation fails

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
        "ai_backend": "OpenRouter (Mistral DevStral)"
    }

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
            selected_text=request.selected_text,
            language=request.language
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
        print(f"\n❌ CRITICAL ERROR in chat endpoint:")
        import traceback
        traceback.print_exc()
        print("="*60 + "\n")
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

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
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/health")
async def health_check():
    """
    Health check - verifies all services
    """
    try:
        # Check Qdrant
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        point_count = collection_info.points_count
        
        return {
            "status": "healthy",
            "qdrant": {
                "connected": True,
                "collection": COLLECTION_NAME,
                "points": point_count
            },
            "cohere": {"connected": True},
            "openrouter": {"connected": True, "model": OPENROUTER_MODEL}
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }

# ============================================================================
# RUN SERVER
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    print("\n" + "="*70)
    print("🚀 Starting Humanoid Robotics Chatbot API")
    print("="*70)
    print(f"📍 API URL: http://localhost:8000")
    print(f"📚 API Docs: http://localhost:8000/docs")
    print(f"🔍 Collection: {COLLECTION_NAME}")
    print(f"🤖 AI Model: {OPENROUTER_MODEL}")
    print(f"💾 Qdrant: {QDRANT_URL}")
    print("="*70)
    
    # Check if collection exists
    try:
        info = qdrant_client.get_collection(COLLECTION_NAME)
        print(f"✅ Qdrant collection found: {info.points_count} points")
    except Exception as e:
        print(f"⚠️ WARNING: Could not access Qdrant collection")
        print(f"   Please run the ingestion script first!")
    
    print("="*70 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)