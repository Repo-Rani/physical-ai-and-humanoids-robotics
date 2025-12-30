import os
import sys
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchText
<<<<<<< HEAD
from dotenv import load_dotenv
=======
>>>>>>> 001-multilingual-chatbot

# ============================================
# CONFIGURATION - Your credentials
# ============================================
<<<<<<< HEAD

load_dotenv()
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY =os.getenv("QDRANT_API_KEY")
=======
COHERE_API_KEY = "jIuKchKF76I843bUTuB0ZR2gSTImVHLlCaYbzUdr"
QDRANT_URL = "https://11e17f22-ead2-4a46-847b-dc4c47d4fff1.europe-west3-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.kOKUvVERRo2H_EXDOF5fv9ajxISlO2gNy6XkBvosh8k"
>>>>>>> 001-multilingual-chatbot
COLLECTION_NAME = "physical_ai_humanoids_robotics"

# ============================================
# CLIENT INITIALIZATION
# ============================================
print("🔄 Initializing Cohere client...")
try:
    cohere_client = cohere.Client(COHERE_API_KEY)
    print("✅ Cohere client initialized successfully\n")
except Exception as e:
    print(f"❌ Failed to initialize Cohere: {e}")
    print("💡 Check your COHERE_API_KEY\n")
    sys.exit(1)

print("🔄 Connecting to Qdrant...")
try:
    qdrant = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=120,
        prefer_grpc=False,
        https=True,
        check_compatibility=False,  # Skip version check to avoid warning
    )
    
    # Test connection immediately
    try:
        collections = qdrant.get_collections()
        print("✅ Qdrant connection successful!")
        
        collection_names = [c.name for c in collections.collections]
        print(f"📦 Available collections: {collection_names}\n")
        
        # Check if our collection exists
        if COLLECTION_NAME not in collection_names:
            print(f"⚠️  Warning: Collection '{COLLECTION_NAME}' not found!")
            print(f"Available: {collection_names}\n")
    except Exception as e:
        print(f"⚠️  Connection test failed: {e}")
        print("\n🔧 Possible issues:")
        print("1. ❌ API key is invalid or expired")
        print("2. ❌ Cluster is paused or inactive")
        print("3. ❌ Wrong cluster URL\n")
        sys.exit(1)
        
except Exception as e:
    print(f"❌ Failed to connect to Qdrant: {e}")
    print("\n🔧 Troubleshooting:")
    print("1. Check if your Qdrant cluster is active")
    print("2. Verify your API key has proper permissions")
    print("3. Generate a new API key from Qdrant dashboard")
    print("4. Update QDRANT_API_KEY at the top of this file\n")
    sys.exit(1)

# ============================================
# CORE FUNCTIONS
# ============================================

def get_embedding(text):
    """Get embedding vector from Cohere with error handling"""
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
    """
    Retrieve relevant chunks with improved parameters and error handling
    
    Args:
        query: User's question
        limit: Maximum number of results (default: 5)
        score_threshold: Minimum similarity score (default: 0.4)
    """
    print(f"\n{'='*70}")
    print(f"🔍 QUERY: {query}")
    print(f"{'='*70}")
    print(f"📊 Parameters: limit={limit}, threshold={score_threshold}\n")
    
    # Get embedding
    embedding = get_embedding(query)
    if embedding is None:
        print("❌ Failed to generate embedding\n")
        return []
    
    # Query Qdrant
    try:
        result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=limit,
            score_threshold=score_threshold
        )
    except Exception as e:
        print(f"❌ Qdrant query failed: {e}")
        print("💡 This might be a 403 Forbidden error - check your API key permissions\n")
        return []
    
    # Process results
    if not result.points:
        print("❌ No results found!")
        print("💡 Try: Lower score_threshold OR rephrase your query\n")
        return []
    
    print(f"✅ Found {len(result.points)} relevant chunks:\n")
    
    chunks = []
    for i, point in enumerate(result.points, 1):
        score = point.score
        text = point.payload.get("text", "")
        url = point.payload.get("url", "N/A")
        title = point.payload.get("page_title", "Unknown")
        
        # Color code by score
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

def retrieve_by_topic(topic_keyword, limit=10):
    """
    Filter by specific topic/page with error handling
    Uses keyword matching in URL field
    
    Args:
        topic_keyword: Keyword to filter URLs (e.g., 'hardware', 'ros2')
        limit: Maximum results
    """
    print(f"\n{'='*70}")
    print(f"📌 FILTERING BY TOPIC: {topic_keyword}")
    print(f"{'='*70}\n")
    
    try:
        # Get all points and filter manually (since URL field doesn't have text index)
        all_results, _ = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            limit=400,  # Get all points (you have 398 total)
            with_payload=True,
            with_vectors=False
        )
        
        # Filter results that contain keyword in URL
        filtered_results = [
            point for point in all_results 
            if topic_keyword.lower() in point.payload.get("url", "").lower()
        ]
        
        # Limit results
        result = filtered_results[:limit]
        
    except Exception as e:
        print(f"❌ Topic filtering failed: {e}\n")
        return []
    
    if not result:
        print(f"❌ No chunks found with '{topic_keyword}' in URL\n")
        return []
    
    print(f"✅ Found {len(result)} chunks about '{topic_keyword}':\n")
    
    for i, point in enumerate(result, 1):
        text = point.payload.get("text", "")
        url = point.payload.get("url", "N/A")
        title = point.payload.get("page_title", "Unknown")
        
        print(f"[{i}] {title}")
        print(f"    🔗 {url.split('/')[-1] if '/' in url else url}")
        print(f"    📝 {text[:120]}...")
        print()
    
    return result

def get_collection_stats():
    """Display collection statistics with error handling"""
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
        print("💡 This is likely a 403 Forbidden error - check API key permissions\n")
        return False

def debug_search(query, limit=20, score_threshold=0.2):
    """
    Debug mode - show MORE results with LOWER threshold
    Use this to see what's actually in the database
    """
    print(f"\n{'='*70}")
    print(f"🐛 DEBUG MODE: {query}")
    print(f"{'='*70}")
    print(f"⚙️  Using low threshold ({score_threshold}) to show more results\n")
    
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
        print(f"❌ Debug search failed: {e}\n")
        return []
    
    if not result.points:
        print("❌ Even with low threshold, nothing found!\n")
        return []
    
    print(f"📊 Found {len(result.points)} results:\n")
    
    for i, point in enumerate(result.points, 1):
        print(f"[{i}] Score: {point.score:.3f} | {point.payload.get('page_title', 'Unknown')}")
    
    print()
    return result.points

def test_specific_pages():
    """Test if specific important pages are indexed"""
    print("\n" + "="*70)
    print("🧪 TESTING SPECIFIC PAGE RETRIEVAL")
    print("="*70 + "\n")
    
    # Updated test topics based on actual URL structure
    test_topics = [
        ("hardware-guide", "Hardware Setup"),
        ("getting-started", "Getting Started"),
        ("module-1-ros2", "ROS2 Module"),
        ("module-2-digital-twin", "Digital Twin/Gazebo"),
        ("module-4-vla", "VLA Module"),
        ("glossary", "Glossary")
    ]
    
    for keyword, description in test_topics:
        try:
            # Get all points and filter manually
            all_results, _ = qdrant.scroll(
                collection_name=COLLECTION_NAME,
                limit=400,  # Get all 398 points
                with_payload=True,
                with_vectors=False
            )
            
            # Filter by keyword in URL
            filtered = [
                point for point in all_results 
                if keyword.lower() in point.payload.get("url", "").lower()
            ]
            
            status = "✅" if filtered else "❌"
            count = len(filtered)
            print(f"{status} {description:30} ({keyword}): {count} chunks")
            
        except Exception as e:
            print(f"❌ {description}: Error - {e}")
    
    print()

def list_all_modules():
    """List all unique modules/pages in the collection"""
    print("\n" + "="*70)
    print("📚 ALL AVAILABLE MODULES AND PAGES")
    print("="*70 + "\n")
    
    try:
        all_results, _ = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            limit=400,
            with_payload=True,
            with_vectors=False
        )
        
        # Extract unique page titles
        unique_pages = {}
        for point in all_results:
            title = point.payload.get("page_title", "Unknown")
            url = point.payload.get("url", "")
            if title not in unique_pages:
                unique_pages[title] = url
        
        print(f"✅ Found {len(unique_pages)} unique pages:\n")
        
        # Group by module
        modules = {}
        for title, url in sorted(unique_pages.items()):
            if "module-" in url.lower():
                # Extract module number
                if "module-0" in url.lower():
                    module = "Module 0: Getting Started"
                elif "module-1" in url.lower():
                    module = "Module 1: ROS2"
                elif "module-2" in url.lower():
                    module = "Module 2: Digital Twin"
                elif "module-3" in url.lower():
                    module = "Module 3: Navigation"
                elif "module-4" in url.lower():
                    module = "Module 4: VLA"
                else:
                    module = "Other Modules"
            else:
                module = "General Pages"
            
            if module not in modules:
                modules[module] = []
            modules[module].append(title)
        
        # Print grouped
        for module, pages in sorted(modules.items()):
            print(f"\n📘 {module} ({len(pages)} pages)")
            print("-" * 70)
            for page in pages[:5]:  # Show first 5
                print(f"  • {page}")
            if len(pages) > 5:
                print(f"  ... and {len(pages) - 5} more")
        
        print()
        
    except Exception as e:
        print(f"❌ Error: {e}\n")

# ============================================
# TEST SUITE
# ============================================

def run_all_tests():
    """Run comprehensive test suite with error handling"""
    
    print("\n" + "🚀 " + "="*66 + " 🚀")
    print("   COMPREHENSIVE RAG RETRIEVAL TEST SUITE")
    print("🚀 " + "="*66 + " 🚀\n")
    
    # Stats first - if this fails, stop
    if not get_collection_stats():
        print("❌ Cannot proceed without collection access")
        print("\n🔧 Fix your credentials:")
        print("1. Go to https://cloud.qdrant.io/")
        print("2. Generate a NEW API key with full permissions")
        print("3. Update QDRANT_API_KEY at line 13 in this file")
        return
    
    # Test 1: Specific technical query
    print("\n📝 TEST 1: Specific Technical Query")
    print("-" * 70)
    retrieve("What are ROS2 nodes and topics?", limit=3, score_threshold=0.5)
    
    # Test 2: Hardware requirements
    print("\n📝 TEST 2: Hardware Requirements (Critical Test!)")
    print("-" * 70)
    retrieve("What hardware do I need for humanoid robotics?", limit=5, score_threshold=0.4)
    
    # Test 3: Prerequisites
    print("\n📝 TEST 3: Course Prerequisites")
    print("-" * 70)
    retrieve("What are the prerequisites for this course?", limit=3, score_threshold=0.5)
    
    # Test 4: Broad conceptual query
    print("\n📝 TEST 4: Broad Conceptual Query")
    print("-" * 70)
    retrieve("Tell me about humanoid robot navigation", limit=5, score_threshold=0.4)
    
    # Test 5: Isaac Sim specific
    print("\n📝 TEST 5: Isaac Sim")
    print("-" * 70)
    retrieve("How does Isaac Sim work?", limit=3, score_threshold=0.5)
    
    # Test 6: Filter by topic
    print("\n📝 TEST 6: Filter by Topic (Hardware)")
    print("-" * 70)
    retrieve_by_topic("hardware", limit=5)
    
    # Test 7: Debug search for hardware
    print("\n📝 TEST 7: Debug Search (Low Threshold)")
    print("-" * 70)
    debug_search("hardware computer GPU", limit=10, score_threshold=0.3)
    
    # Test 8: Check specific pages exist
    test_specific_pages()
    
    # Test 9: List all available modules
    list_all_modules()
    
    print("\n" + "="*70)
    print("✅ TEST SUITE COMPLETE!")
    print("="*70 + "\n")

def quick_test():
    """Quick test for rapid iteration"""
    if not get_collection_stats():
        print("❌ Collection access failed - check credentials")
        return
    
    # Test the previously failing query
    print("\n🎯 QUICK TEST: Hardware Requirements\n")
    results = retrieve("What hardware specifications are needed?", limit=5, score_threshold=0.4)
    
    if len(results) == 0:
        print("⚠️  Still no results! Running debug search...\n")
        debug_search("hardware", limit=10, score_threshold=0.2)

# ============================================
# MAIN
# ============================================

if __name__ == "__main__":
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "quick":
            quick_test()
        elif sys.argv[1] == "debug":
            query = " ".join(sys.argv[2:]) if len(sys.argv) > 2 else "hardware"
            get_collection_stats()
            debug_search(query, limit=20, score_threshold=0.2)
        elif sys.argv[1] == "filter":
            topic = sys.argv[2] if len(sys.argv) > 2 else "hardware"
            get_collection_stats()
            retrieve_by_topic(topic, limit=10)
        elif sys.argv[1] == "modules":
            # New command to list all modules
            get_collection_stats()
            list_all_modules()
        else:
            # Custom query
            query = " ".join(sys.argv[1:])
            get_collection_stats()
            retrieve(query, limit=5, score_threshold=0.4)
    else:
        # Run full test suite
        run_all_tests()