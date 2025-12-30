import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Get Qdrant credentials
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "docusaurus_docs")

print(f"QDRANT_URL: {QDRANT_URL}")
print(f"QDRANT_API_KEY: {'***' + QDRANT_API_KEY[-10:] if QDRANT_API_KEY else 'NOT SET'}")
print(f"COLLECTION_NAME: {COLLECTION_NAME}")
print("-" * 50)

try:
    # Connect to Qdrant
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=False
    )
    
    print("✓ Connected to Qdrant successfully!")
    
    # List all collections
    collections = client.get_collections()
    print(f"\nAvailable collections: {[c.name for c in collections.collections]}")
    
    # Check if our collection exists
    if COLLECTION_NAME in [c.name for c in collections.collections]:
        print(f"\n✓ Collection '{COLLECTION_NAME}' exists!")
        
        # Get collection info
        collection_info = client.get_collection(COLLECTION_NAME)
        print(f"  - Points count: {collection_info.points_count}")
        print(f"  - Vector size: {collection_info.config.params.vectors.size}")
    else:
        print(f"\n✗ Collection '{COLLECTION_NAME}' does NOT exist!")
        print("  You need to run the ingestion pipeline to create and populate the collection.")
        
except Exception as e:
    print(f"\n✗ Error connecting to Qdrant: {type(e).__name__}: {str(e)}")
    import traceback
    traceback.print_exc()
