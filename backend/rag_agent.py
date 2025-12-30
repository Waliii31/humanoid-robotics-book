import asyncio
import logging
from typing import List, Dict, Any, Tuple, Optional
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
import json
from pydantic import BaseModel, Field
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
import os
from openai import OpenAI

def get_openai_client():
    """Create and return an OpenAI client configured for Gemini API"""
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    if not gemini_api_key:
        raise ValueError("GEMINI_API_KEY environment variable is not set")

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
        prefer_grpc=False
    )

QDRANT_URL = os.getenv("QDRANT_URL", "https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "docusaurus_docs")

class RetrievalTool(BaseModel):
    """Tool for retrieving documentation chunks from vector store"""
    name: str = "retrieve_documentation"
    description: str = "Retrieve relevant documentation chunks based on user query"
    parameters: Dict[str, Any] = Field(default={
        "type": "object",
        "properties": {
            "query": {"type": "string", "description": "The user's query to search for relevant documentation"},
            "top_k": {"type": "integer", "description": "Number of chunks to retrieve", "default": 5}
        },
        "required": ["query"]
    })

async def retrieve_chunks(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve relevant chunks from Qdrant based on the query
    """
    try:
        # First check if the collection exists
        qdrant_client = get_qdrant_client()
        
        try:
            collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            if collection_info.points_count == 0:
                logger.warning(f"Collection '{QDRANT_COLLECTION_NAME}' exists but is empty")
                return []
        except Exception as collection_error:
            logger.error(f"Collection '{QDRANT_COLLECTION_NAME}' does not exist: {str(collection_error)}")
            return []
        
        # Generate embedding for the query using Gemini
        client = get_openai_client()
        response = client.embeddings.create(
            input=query,
            model="text-embedding-005"  # Using Google's embedding model compatible with Gemini API
        )
        query_embedding = response.data[0].embedding

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Format results
        chunks = []
        for result in search_results:
            payload = result.payload
            chunk = {
                "id": str(result.id),
                "content": payload.get("content", ""),
                "url": payload.get("url", ""),
                "heading": payload.get("heading", ""),
                "chunk_index": payload.get("chunkIndex", 0),
                "metadata": payload.get("metadata", {}),
                "score": result.score
            }
            chunks.append(chunk)

        return chunks
    except Exception as e:
        logger.error(f"Error retrieving chunks: {str(e)}")
        import traceback
        traceback.print_exc()
        return []

async def execute_retrieval_tool(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """Execute the retrieval tool to get relevant documentation"""
    return await retrieve_chunks(query, top_k)

class RagAgent:
    """RAG Agent that orchestrates retrieval and response generation"""

    def __init__(self, temperature: float = 0.1):
        self.temperature = temperature
        self.tools = [RetrievalTool()]

    async def process_query(
        self,
        query: str,
        user_selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> Tuple[str, List[Dict[str, Any]], Dict[str, int]]:
        """
        Process a user query using RAG approach
        """
        try:
            if user_selected_text:
                # If user provided specific text, use that as context
                retrieved_chunks = [{
                    "id": "user_provided",
                    "content": user_selected_text,
                    "url": "user_input",
                    "heading": "User Provided Context",
                    "chunk_index": 0,
                    "metadata": {},
                    "score": 1.0
                }]
            else:
                # Use tool to retrieve from vector store
                retrieved_chunks = await execute_retrieval_tool(query, top_k)

            # Prepare context from retrieved chunks
            context_parts = []
            for chunk in retrieved_chunks:
                content = chunk["content"]
                if content.strip():  # Only add non-empty content
                    context_parts.append(f"Source: {chunk['url']}\nContent: {content}")

            if not context_parts:
                # No documentation found - provide a helpful message
                return (
                    "I'm sorry, but the documentation database is currently empty. "
                    "The vector store needs to be populated with documentation content first. "
                    "Please run the ingestion pipeline to index the textbook content into Qdrant.",
                    [],
                    {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}
                )
            
            context = "\n\n".join(context_parts)

            # Create the system message to ground the response in documentation
            system_message = f"""You are a helpful documentation assistant for the 'Physical AI & Humanoid Robotics' textbook.
Your responses must be grounded strictly in the provided documentation context.
Do not hallucinate or provide information beyond what is in the context.
If the context doesn't contain the information needed to answer the query, say so explicitly.
Always cite the source URL when referencing specific documentation."""

            # Create the user message with context
            user_message = f"""Based on the following documentation context, please answer the user's query:

DOCUMENTATION CONTEXT:
{context}

USER QUERY:
{query}

ANSWER:"""

            # Call Gemini API to generate response
            client = get_openai_client()
            response = client.chat.completions.create(
                model="gemini-2.0-flash",  # Using Gemini flash model
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                temperature=self.temperature,
                max_tokens=1000
            )

            # Extract the response
            generated_response = response.choices[0].message.content

            # Calculate token usage
            token_usage = {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens
            }

            return generated_response, retrieved_chunks, token_usage

        except Exception as e:
            logger.error(f"Error in RAG agent: {str(e)}")
            error_msg = "Sorry, I encountered an error processing your request. Please try again."
            return error_msg, [], {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}

# Create a global agent instance
rag_agent = RagAgent()

async def rag_agent_response(
    query: str,
    user_selected_text: Optional[str] = None,
    top_k: int = 5,
    temperature: float = 0.1
) -> Tuple[str, List[Dict[str, Any]], Dict[str, int]]:
    """
    Main RAG agent function that retrieves context and generates response
    """
    # Update agent temperature if needed
    if temperature != rag_agent.temperature:
        rag_agent.temperature = temperature

    return await rag_agent.process_query(query, user_selected_text, top_k)

async def validate_retrieval_quality(query: str, expected_keywords: List[str] = None) -> Dict[str, Any]:
    """
    Validate the quality of retrieval for a given query
    """
    if expected_keywords is None:
        expected_keywords = []

    retrieved_chunks = await retrieve_chunks(query, top_k=5)

    # Basic validation: check if any expected keywords appear in retrieved content
    found_keywords = []
    if expected_keywords:
        content_text = " ".join([chunk["content"] for chunk in retrieved_chunks]).lower()
        found_keywords = [kw for kw in expected_keywords if kw.lower() in content_text]

    return {
        "query": query,
        "retrieved_chunks_count": len(retrieved_chunks),
        "expected_keywords": expected_keywords,
        "found_keywords": found_keywords,
        "keyword_match_ratio": len(found_keywords) / len(expected_keywords) if expected_keywords else 0,
        "chunks": retrieved_chunks
    }