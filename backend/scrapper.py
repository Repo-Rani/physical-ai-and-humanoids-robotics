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
COLLECTION_NAME = "physical_ai_humanoids_robotics"

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