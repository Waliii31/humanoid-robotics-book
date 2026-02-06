# 🚀 Quick Start - 5 Minutes Setup

## TL;DR - Get RAG Working Now

### 1️⃣ Get Free API Keys (2 min)

**Qdrant Vector DB:**
- Go to https://qdrant.tech/
- Sign up → Create free cloud cluster
- Copy API URL and key

**Cohere Embeddings:**
- Go to https://cohere.com/ 
- Sign up → Create API key
- Copy key (free tier available)

**Gemini AI:**
- Go to https://ai.google.dev/
- Click "Get API Key"
- Copy key (free limited access)

### 2️⃣ Configure Backend (1 min)

```bash
cd backend
cp .env.example .env
```

Edit `.env` and paste your API keys:
```
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_key
COHERE_API_KEY=your_key
GEMINI_API_KEY=your_key
```

### 3️⃣ Ingest Documentation (1 min)

```bash
cd backend
npm install          # First time only
npm run ingest       # Wait for completion
```

You'll see: `✅ Successfully indexed 100+ chunks into Qdrant!`

### 4️⃣ Start Services (1 min)

**Terminal 1:**
```bash
npm start            # From root directory
# Opens http://localhost:3000
```

**Terminal 2:**
```bash
cd backend
npm run dev          # Separate terminal
# Backend at http://localhost:8000
```

### 5️⃣ Use Chatbot! 🎉

Open http://localhost:3000 → Look for "Documentation Assistant" → Ask questions!

---

## Commands Quick Reference

```bash
# Diagnostics
cd backend && npm run diagnose    # Check if everything is configured

# Setup
cd backend && npm install         # Install dependencies (once)
cd backend && npm run ingest      # Index documents (once)

# Running
npm start                         # Frontend (from root)
cd backend && npm run dev         # Backend (separate terminal)

# Testing
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?"}'
```

---

## ✅ Verification Checklist

- [ ] API keys obtained from Qdrant, Cohere, Gemini
- [ ] `.env` file created and filled with keys
- [ ] `npm install` completed in backend/
- [ ] `npm run ingest` completed successfully
- [ ] Frontend running at http://localhost:3000
- [ ] Backend running at http://localhost:8000
- [ ] Chatbot responds to questions

---

## 🆘 If Still Not Working

```bash
# 1. Check configuration
cd backend
npm run diagnose

# 2. Reinstall and retry ingestion
rm -rf node_modules
npm install
npm run ingest

# 3. Check API key validity in .env
# Make sure they're real, not examples

# 4. Restart everything
# Kill both terminals, start fresh
```

---

**That's it! Enjoy your AI-powered documentation! 🚀**
