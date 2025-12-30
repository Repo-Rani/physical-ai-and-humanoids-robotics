from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import set_tracing_disabled, function_tool, enable_verbose_stdout_logging
import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

load_dotenv()
# set_tracing_disabled(disabled=True)  # Temporarily disable this for debugging
# enable_verbose_stdout_logging()  # Enable this to see what's happening

openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
provider = AsyncOpenAI(
    api_key=openrouter_api_key,
    base_url="https://openrouter.ai/api/v1"
)

model = OpenAIChatCompletionsModel(
    model="mistralai/devstral-2512:free",
    openai_client=provider
)

cohere_client = cohere.Client("jIuKchKF76I843bUTuB0ZR2gSTImVHLlCaYbzUdr")
qdrant = QdrantClient(
    url="https://11e17f22-ead2-4a46-847b-dc4c47d4fff1.europe-west3-0.gcp.cloud.qdrant.io",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.kOKUvVERRo2H_EXDOF5fv9ajxISlO2gNy6XkBvosh8k"
)



def get_embedding(text):
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

# ========== DEBUGGING STARTS HERE ==========

print("="*60)
print("STEP 1: Checking Collection")
print("="*60)

try:
    collection_info = qdrant.get_collection("physical_ai_humanoids_robotics")
    print(f"✓ Collection exists")
    print(f"  Total documents: {collection_info.points_count}")
    print(f"  Vector size: {collection_info.config.params.vectors.size}")
    
    if collection_info.points_count == 0:
        print("\n❌ ERROR: Collection is EMPTY!")
        print("You need to upload documents first.")
        exit()
        
except Exception as e:
    print(f"❌ ERROR: Cannot access collection: {e}")
    exit()

print("\n" + "="*60)
print("STEP 2: Testing Direct Retrieval")
print("="*60)

query = "what is physical ai?"
print(f"Query: '{query}'")

embedding = get_embedding(query)
print(f"Embedding size: {len(embedding)}")

result = qdrant.query_points(
    collection_name="physical_ai_humanoids_robotics",
    query=embedding,
    limit=5
)

print(f"\nRetrieved {len(result.points)} documents:")
for i, point in enumerate(result.points):
    print(f"\n{i+1}. Score: {point.score:.4f}")
    print(f"   Text: {point.payload.get('text', 'NO TEXT')[:150]}...")

if len(result.points) == 0:
    print("\n❌ ERROR: No documents retrieved!")
    print("This means either:")
    print("  1. Collection is empty")
    print("  2. Query embedding doesn't match document embeddings")
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
        collection_name="physical_ai_humanoids_robotics",
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