from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
import os
import asyncio
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Documentation Chatbot API",
    description="A RAG-capable backend agent for documentation-based Q&A",
    version="1.0.0"
)

# Add CORS middleware to allow frontend to communicate with backend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus dev server
        "http://127.0.0.1:3000",
        "http://localhost:8000",  # Backend itself
        "http://127.0.0.1:8000",
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "docusaurus_docs")

# Initialize clients
from openai import OpenAI

def get_openai_client():
    """Create and return an OpenAI client configured for Gemini API"""
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    if not gemini_api_key:
        raise ValueError("GEMINI_API_KEY environment variable is not set")

    # Initialize the OpenAI client with the Gemini API configuration
    # Using the OpenAI-compatible endpoint for Gemini
    return OpenAI(
        api_key=gemini_api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

def get_qdrant_client():
    """Create and return a Qdrant client"""
    qdrant_url = os.getenv("QDRANT_URL", "https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY environment variable is required")

    return QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False  # Using REST API
    )

# Pydantic models
class Message(BaseModel):
    role: str = Field(..., description="The role of the message sender (user or assistant)")
    content: str = Field(..., description="The content of the message")

class ChatRequest(BaseModel):
    messages: List[Message] = Field(..., description="List of conversation messages")
    top_k: int = Field(5, ge=1, le=20, description="Number of chunks to retrieve")
    temperature: float = Field(0.1, ge=0.0, le=2.0, description="Temperature for response generation")
    user_selected_text: Optional[str] = Field(None, description="User-provided text to use as context")

class ChatResponse(BaseModel):
    response: str = Field(..., description="The agent's response")
    retrieved_chunks: List[Dict[str, Any]] = Field(..., description="Chunks retrieved for context")
    tokens_used: Dict[str, int] = Field(..., description="Token usage information")

class HealthResponse(BaseModel):
    status: str = Field(..., description="Health status of the service")
    services: Dict[str, bool] = Field(..., description="Status of dependent services")

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint to verify service status"""
    try:
        # Test Gemini connection
        gemini_ok = False
        try:
            client = get_openai_client()
            client.models.list()
            gemini_ok = True
        except Exception:
            pass

        # Test Qdrant connection
        qdrant_ok = False
        try:
            qdrant_client = get_qdrant_client()
            qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            qdrant_ok = True
        except Exception:
            pass

        all_ok = gemini_ok and qdrant_ok
        status = "healthy" if all_ok else "unhealthy"

        return HealthResponse(
            status=status,
            services={
                "gemini": gemini_ok,
                "qdrant": qdrant_ok
            }
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            services={
                "openai": False,
                "qdrant": False
            }
        )

# Import the RAG agent functionality
from rag_agent import rag_agent_response

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint that processes user queries using RAG
    """
    if not GEMINI_API_KEY:
        raise HTTPException(status_code=500, detail="Gemini API key not configured")

    try:
        # Get the last user message as the query
        user_messages = [msg for msg in request.messages if msg.role == "user"]
        if not user_messages:
            raise HTTPException(status_code=400, detail="No user message found in request")

        query = user_messages[-1].content

        # Call the RAG agent
        response, retrieved_chunks, token_usage = await rag_agent_response(
            query=query,
            user_selected_text=request.user_selected_text,
            top_k=request.top_k,
            temperature=request.temperature
        )

        return ChatResponse(
            response=response,
            retrieved_chunks=retrieved_chunks,
            tokens_used=token_usage
        )
    except Exception as e:
        logger.error(f"Chat endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.get("/")
async def root():
    """Root endpoint for basic service information"""
    return {
        "message": "RAG Documentation Chatbot API",
        "status": "running",
        "endpoints": {
            "chat": "/chat (POST)",
            "health": "/health (GET)"
        }
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)