import asyncio
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import the RAG agent
from rag_agent import rag_agent_response

async def test_chatbot():
    """Test the chatbot with a simple query"""
    output_file = "test_output.txt"
    
    with open(output_file, "w", encoding="utf-8") as f:
        try:
            f.write("Testing RAG chatbot...\n")
            f.write("-" * 50 + "\n")
            
            query = "What is this textbook about?"
            f.write(f"Query: {query}\n")
            f.write("-" * 50 + "\n")
            
            response, chunks, tokens = await rag_agent_response(
                query=query,
                top_k=5,
                temperature=0.1
            )
            
            f.write(f"\nResponse: {response}\n")
            f.write(f"\nRetrieved chunks: {len(chunks)}\n")
            f.write(f"Tokens used: {tokens}\n")
            
            if chunks:
                f.write("\nFirst chunk:\n")
                f.write(str(chunks[0]) + "\n")
            
            print(f"Test completed! Output written to {output_file}")
            print(f"Response: {response}")
            
        except Exception as e:
            error_msg = f"ERROR: {type(e).__name__}: {str(e)}\n"
            f.write(error_msg)
            print(error_msg)
            import traceback
            traceback_str = traceback.format_exc()
            f.write(traceback_str)
            print(traceback_str)

if __name__ == "__main__":
    asyncio.run(test_chatbot())
