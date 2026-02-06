# Fix: RAG Chatbot - "Documentation Database is Empty"

## Problem

When you interact with the Documentation Assistant chatbot, you get:
```
I'm sorry, but the documentation database is currently empty. 
The vector store needs to be populated with documentation content first. 
Please run the ingestion pipeline to index the textbook content into Qdrant.
```

## Root Cause

The Qdrant vector database doesn't have any indexed documents. You need to:
1. Configure your API credentials (Qdrant, Cohere, Gemini)
2. Run the ingestion pipeline to index the local markdown documentation

## Solution (5-Step Quick Start)

### Step 1: Install Backend Dependencies

```bash
cd backend
npm install
```

### Step 2: Configure API Keys

Create a `.env` file in the `backend/` directory:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```env
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_api_key
COHERE_API_KEY=your_cohere_key
GEMINI_API_KEY=your_gemini_key
```

**Where to get free API keys:**
- **Qdrant**: https://qdrant.tech/ (Free cloud cluster)
- **Cohere**: https://cohere.com/ (Free trial: 100 API calls/month)
- **Gemini**: https://ai.google.dev/ (Free limited API)

### Step 3: Verify Configuration

Run the diagnostic check:

```bash
npm run diagnose
```

This will verify:
- ✅ `.env` file exists and is readable
- ✅ All API credentials are set
- ✅ Qdrant cluster is accessible
- ✅ Node dependencies are installed
- ✅ Documentation files are available

### Step 4: Ingest Documentation (Important!)

```bash
npm run ingest
```

This reads all 13 chapters from the `../docs` directory and indexes them into Qdrant.

**Expected output:**
```
🚀 Starting Local Markdown Ingestion...

✓ Collection "docusaurus_docs" already exists

📖 Reading markdown files from: ../docs

✓ Loaded: week01-02-physical-ai/01-foundations-of-physical-ai.md
✓ Loaded: week01-02-physical-ai/02-embodied-intelligence-architecture.md
... (all 13 chapters)

✓ Found 13 documents

📚 Processing 13 documents...
  Foundations of Physical AI: 8 chunks
  Embodied Intelligence Architecture: 7 chunks
  ... 

📤 Uploading 100+ chunks to Qdrant...
  ✓ Uploaded 50/100
  ✓ Uploaded 100/100

✅ Successfully indexed 100+ chunks into Qdrant!

✨ Ingestion complete!
```

### Step 5: Start the Services

**Terminal 1 - Start Frontend:**
```bash
# From project root
npm start
```
Docusaurus will start at http://localhost:3000

**Terminal 2 - Start Backend:**
```bash
# From backend/ directory
npm run dev
```
Backend will start at http://localhost:8000

## Testing the Chatbot

1. Open http://localhost:3000 in your browser
2. Look for the "Documentation Assistant" widget (bottom right or integrated in the page)
3. Try asking a question:
   - "What is Physical AI?"
   - "How do I set up ROS 2?"
   - "Explain bipedal locomotion"
   - "What are the hardware requirements?"

## Troubleshooting

### ❌ "Missing required environment variables"

**Solution:**
```bash
cp .env.example .env
# Edit .env and add your actual API keys
```

### ❌ "Cannot connect to Qdrant"

**Solutions:**
1. Verify `QDRANT_URL` is correct in `.env`
2. Check API key is valid
3. Ensure Qdrant cluster is running
4. Test connectivity: `npm run diagnose`

### ❌ "COHERE_API_KEY not found"

**Solution:**
1. Get a free Cohere API key: https://cohere.com/
2. Add to `.env`: `COHERE_API_KEY=your_key`

### ❌ "No markdown files found"

**Solution:**
Make sure you're running from the `backend/` directory:
```bash
cd backend
npm run ingest
```

### ❌ "Ingestion is very slow"

**Normal behavior** - First ingestion takes time because it:
1. Reads all 13 chapters
2. Splits them into ~100+ chunks
3. Generates embeddings via Cohere API
4. Uploads to Qdrant

**To speed up:**
- Use `COHERE_MODEL=embed-english-light-v3.0` for faster (less accurate) embeddings
- Increase `EMBEDDING_BATCH_SIZE` in `.env` (if your API plan allows)

### ✅ "Ingestion succeeded but chatbot still shows empty database"

**Solution:**
1. Refresh the browser (Ctrl+F5 or Cmd+Shift+R)
2. Restart the backend: `npm run dev`
3. Wait 5-10 seconds for the API to initialize

## Backend Scripts Reference

| Command | Purpose |
|---------|---------|
| `npm run diagnose` | Check configuration and connectivity |
| `npm run ingest` | Index local docs into Qdrant (required first!) |
| `npm run dev` | Start backend with auto-reload |
| `npm run build` | Compile TypeScript |
| `npm start` | Run compiled backend |

## Project Structure

```
backend/
├── src/
│   ├── embedder.ts       # Cohere embeddings integration
│   ├── chunker.ts        # Document chunking
│   ├── vector-store.ts   # Qdrant client
│   ├── retriever.ts      # Semantic search
│   └── ...
├── ingest-local.ts       # ⭐ Local markdown ingestion
├── diagnose.ts           # ⭐ Configuration diagnostic
├── main.py               # FastAPI backend
├── .env.example          # Example configuration
├── SETUP_GUIDE.md        # Detailed setup guide
└── package.json
```

## Next Steps

1. ✅ Follow Step 1-5 above
2. ✅ Confirm ingestion completes successfully
3. ✅ Ask the chatbot about the curriculum
4. ✅ Enjoy the integrated Documentation Assistant!

## Need More Help?

- See [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed backend setup
- Check [../README.md](../README.md) for full project documentation
- Review `.env.example` for all configuration options

---

**Once your API keys are configured and ingestion is complete, the chatbot will work perfectly!** 🎉
