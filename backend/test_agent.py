import asyncio
import os
from rag_agent import rag_agent_response, validate_retrieval_quality

async def test_agent():
    """Test the RAG agent functionality"""
    print("Testing RAG Agent...\n")

    # Test 1: Basic query
    print("1. Testing basic query...")
    try:
        response, chunks, tokens = await rag_agent_response(
            query="What is physical AI?",
            top_k=3
        )
        print(f"   Response: {response[:200]}...")
        print(f"   Retrieved {len(chunks)} chunks")
        print(f"   Tokens used: {tokens}")
        print("   ✓ Basic query test passed\n")
    except Exception as e:
        print(f"   ✗ Basic query test failed: {e}\n")

    # Test 2: Query with user-provided text
    print("2. Testing with user-provided context...")
    try:
        user_text = "Physical AI is an approach to robotics that emphasizes the physical interaction between robots and their environment. It combines principles of physics, machine learning, and control theory."
        response, chunks, tokens = await rag_agent_response(
            query="Explain the concept of Physical AI",
            user_selected_text=user_text
        )
        print(f"   Response: {response[:200]}...")
        print(f"   Retrieved {len(chunks)} chunks (should be 1 from user text)")
        print(f"   Tokens used: {tokens}")
        print("   ✓ User-provided context test passed\n")
    except Exception as e:
        print(f"   ✗ User-provided context test failed: {e}\n")

    # Test 3: Retrieval validation
    print("3. Testing retrieval validation...")
    try:
        validation_result = await validate_retrieval_quality(
            query="humanoid robotics fundamentals",
            expected_keywords=["humanoid", "robotics", "control", "locomotion"]
        )
        print(f"   Query: {validation_result['query']}")
        print(f"   Retrieved {validation_result['retrieved_chunks_count']} chunks")
        print(f"   Expected keywords: {validation_result['expected_keywords']}")
        print(f"   Found keywords: {validation_result['found_keywords']}")
        print(f"   Match ratio: {validation_result['keyword_match_ratio']:.2f}")
        print("   ✓ Retrieval validation test passed\n")
    except Exception as e:
        print(f"   ✗ Retrieval validation test failed: {e}\n")

    # Test 4: Edge case - empty query
    print("4. Testing edge case with empty query...")
    try:
        response, chunks, tokens = await rag_agent_response(
            query="",
            top_k=1
        )
        print(f"   Response: {response[:100]}...")
        print(f"   Retrieved {len(chunks)} chunks")
        print("   ✓ Empty query test passed\n")
    except Exception as e:
        print(f"   ✗ Empty query test failed: {e}\n")

    # Test 5: Complex query
    print("5. Testing complex query...")
    try:
        response, chunks, tokens = await rag_agent_response(
            query="Compare simulation-based training vs real-world training for humanoid robots",
            top_k=5,
            temperature=0.3
        )
        print(f"   Response: {response[:300]}...")
        print(f"   Retrieved {len(chunks)} chunks")
        print(f"   Tokens used: {tokens}")
        print("   ✓ Complex query test passed\n")
    except Exception as e:
        print(f"   ✗ Complex query test failed: {e}\n")

    print("Agent testing completed!")

if __name__ == "__main__":
    if not os.getenv("GEMINI_API_KEY"):
        print("Warning: GEMINI_API_KEY not set. Tests may fail.")

    if not os.getenv("QDRANT_URL"):
        print("Warning: QDRANT_URL not set. Tests may fail.")

    asyncio.run(test_agent())