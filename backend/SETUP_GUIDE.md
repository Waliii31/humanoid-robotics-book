# RAG Backend Setup & Configuration Guide

## Quick Start

### 1. Install Dependencies

```bash
cd backend
npm install
```

### 2. Configure Environment Variables

```bash
# Copy the example file
cp .env.example .env

# Edit .env with your API keys
nano .env  # or use your preferred editor
```

### 3. Get API Keys

#### **Qdrant Vector Database** (Free Cloud Option)
1. Go to https://qdrant.tech/
2. Sign up and create a free cloud cluster
3. Get your API URL and key from the dashboard
4. Add to `.env`:
   ```
   QDRANT_URL=https://your-cluster.cloud.qdrant.io
   QDRANT_API_KEY=your_key_here
   ```

#### **Cohere Embeddings API** (Free Trial Available)
1. Go to https://cohere.com/
2. Sign up for a free account
3. Go to API keys section and create one
4. Add to `.env`:
   ```
   COHERE_API_KEY=your_key_here
   ```

#### **Gemini API** (Google - Free Option)
1. Go to https://ai.google.dev/
2. Click "Get API Key"
3. Create a new API key in Google Cloud Console
4. Add to `.env`:
   ```
   GEMINI_API_KEY=your_key_here
   ```

---

## Ingest Documentation into Qdrant

Before using the chatbot, you must index the documentation.

### Option 1: Ingest Local Markdown Files (Recommended)

```bash
cd backend
npm run ingest
```

This reads all markdown files from the `../docs` directory and indexes them into Qdrant.

**Expected Output:**
```
🚀 Starting Local Markdown Ingestion...

✓ Collection "docusaurus_docs" already exists

📖 Reading markdown files from: ../docs

✓ Loaded: week01-02-physical-ai/01-foundations-of-physical-ai.md
✓ Loaded: week01-02-physical-ai/02-embodied-intelligence-architecture.md
... (all other chapters)

✓ Found 13 documents

📚 Processing 13 documents...
  Foundations of Physical AI: 8 chunks
  Embodied Intelligence Architecture: 7 chunks
  ... (all documents)

📤 Uploading 100 chunks to Qdrant...
  ✓ Uploaded 50/100
  ✓ Uploaded 100/100

✅ Successfully indexed 100 chunks into Qdrant!

✨ Ingestion complete!
```

### Option 2: Crawl Deployed Website

```bash
cd backend
npm run crawl
```

Requires `DEPLOYED_SITE_URL` set in `.env` pointing to your live Docusaurus site.

---

## Run the Backend Server

### Development Mode (with auto-reload)

```bash
cd backend
npm run dev
```

Server will start at `http://localhost:8000`

### Production Mode

```bash
cd backend
npm run build
npm start
```

---

## Run Frontend + Backend Together

### Terminal 1: Frontend (Docusaurus)
```bash
npm start
```
Docusaurus dev server at `http://localhost:3000`

### Terminal 2: Backend RAG Pipeline  
```bash
cd backend
npm run dev
```
Backend server at `http://localhost:8000`

---

## Verify the Setup

### Test the RAG Chatbot

1. Open http://localhost:3000 in your browser
2. Look for the "Documentation Assistant" chatbot widget
3. Ask a question about the curriculum, e.g.:
   - "What is Physical AI?"
   - "How do I set up ROS 2?"
   - "Explain bipedal locomotion"

### Test the API Directly

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "top_k": 5
  }'
```

Expected response:
```json
{
  "answer": "Physical AI is...",
  "sources": [
    {
      "title": "Foundations of Physical AI",
      "url": "/docs/week01-02-physical-ai/01-foundations-of-physical-ai",
      "excerpt": "..."
    }
  ]
}
```

---

## Troubleshooting

### Issue: "Documentation database is currently empty"

**Solution:** Run the ingestion pipeline:
```bash
npm run ingest
```

### Issue: "QDRANT_URL is not set"

**Solution:** Create `.env` file with proper configuration:
```bash
cp .env.example .env
nano .env  # Add your Qdrant URL and API key
```

### Issue: "COHERE_API_KEY not found"

**Solution:** Add your Cohere API key to `.env`:
```
COHERE_API_KEY=your_actual_key_here
```

### Issue: Backend won't start

**Solutions:**
1. Check all required env vars are set: `QDRANT_URL`, `COHERE_API_KEY`, `GEMINI_API_KEY`
2. Verify Qdrant cluster is accessible
3. Check Node.js version: `node --version` (should be 14+)
4. Clear cache: `rm -rf node_modules && npm install`

### Issue: Slow embeddings generation

This is normal for the first ingestion. Cohere's API processes chunks sequentially.
- Skip non-essential content to speed up
- Or use `EMBEDDING_BATCH_SIZE=64` in `.env` to process faster (requires higher rate limits)

---

## Project Structure

```
backend/
├── src/
│   ├── crawler.ts           # Web crawler for documentation
│   ├── chunker.ts           # Document chunking strategy
│   ├── embedder.ts          # Cohere embeddings integration
│   ├── vector-store.ts      # Qdrant vector database client
│   ├── retriever.ts         # Semantic search & retrieval
│   ├── pipeline.ts          # RAG pipeline orchestration
│   ├── types.ts             # TypeScript types
│   ├── index.ts             # Main entry point
│   └── test.ts              # Test utilities
├── main.py                  # FastAPI REST API server
├── rag_agent.py             # RAG agent logic
├── ingest-local.ts          # Local markdown ingestion ⭐ NEW
├── .env.example             # Example environment configuration
├── package.json
├── tsconfig.json
└── requirements.txt         # Python dependencies
```

---

## Available Scripts

| Command | Purpose |
|---------|---------|
| `npm run dev` | Run TypeScript backend with auto-reload |
| `npm run build` | Compile TypeScript to JavaScript |
| `npm start` | Run compiled backend |
| `npm run ingest` | Ingest local markdown docs into Qdrant ⭐ |
| `npm run crawl` | Crawl an external website (requires `DEPLOYED_SITE_URL`) |
| `npm run test` | Run test suite |

---

## Next Steps

1. ✅ Configure `.env` with your API keys
2. ✅ Run `npm run ingest` to populate Qdrant
3. ✅ Start frontend: `npm start` (from root)
4. ✅ Start backend: `npm run dev` (from backend/)
5. ✅ Test chatbot in the UI
6. ✅ Ask questions about the curriculum!

---

## Performance Tips

- **First ingestion is slow**: This is expected while generating embeddings
- **Use Cohere's faster model**: Set `COHERE_MODEL=embed-english-light-v3.0` for faster (but less accurate) embeddings
- **Batch processing**: Adjust `EMBEDDING_BATCH_SIZE` based on your API rate limits
- **Caching**: Add Redis to cache frequently asked questions

---

## Additional Resources

- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Cohere API**: https://docs.cohere.com/
- **Gemini API**: https://ai.google.dev/docs
- **FastAPI**: https://fastapi.tiangolo.com/
- **Docusaurus**: https://docusaurus.io/

---

**Need Help?**
- Check `.env` file is configured correctly
- Verify all API keys are valid and have sufficient quota
- Check network connectivity to Qdrant Cloud
- Review console logs for detailed error messages
