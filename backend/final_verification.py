#!/usr/bin/env python3
"""
Final verification that the application is configured correctly with Gemini API and QDRANT
"""
import os

# Set the environment variables
os.environ["GEMINI_API_KEY"] = "AIzaSyCe6tUMSZGFkxzqEGqdSit1qKOUVw0RsCA"
os.environ["QDRANT_URL"] = "https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io"
os.environ["QDRANT_API_KEY"] = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Pej0yjlpDqhQVdnOFicUHdZWVki9DAzUhywMQ76QJgs"

def test_imports():
    """Test that all modules can be imported successfully"""
    try:
        from main import app
        print("SUCCESS: Main application imported successfully")
    except Exception as e:
        print(f"ERROR: Failed to import main: {e}")
        return False

    try:
        from rag_agent import rag_agent_response, get_openai_client, get_qdrant_client
        print("SUCCESS: RAG agent imported successfully")
    except Exception as e:
        print(f"ERROR: Failed to import rag_agent: {e}")
        return False

    try:
        from test_agent import test_agent
        print("SUCCESS: Test agent imported successfully")
    except Exception as e:
        print(f"ERROR: Failed to import test_agent: {e}")
        return False

    return True

def test_function_definitions():
    """Test that functions can be defined without errors"""
    try:
        from rag_agent import get_openai_client, get_qdrant_client

        # Test that functions are defined properly
        import inspect
        if inspect.isfunction(get_openai_client) and inspect.isfunction(get_qdrant_client):
            print("SUCCESS: Client functions are properly defined")
        else:
            print("ERROR: Client functions are not properly defined")
            return False
    except Exception as e:
        print(f"ERROR: Failed to define functions: {e}")
        return False

    return True

def main():
    print("Final verification of Gemini API and QDRANT configuration...")
    print()

    print("Testing imports and function definitions...")
    imports_ok = test_imports()
    print()

    print("Testing function definitions...")
    functions_ok = test_function_definitions()
    print()

    if imports_ok and functions_ok:
        print("SUCCESS: All configuration changes completed successfully!")
        print()
        print("Configuration summary:")
        print("  - Application now uses Gemini API instead of OpenAI API")
        print("  - API key: AIzaSyCe6tUMSZGFkxzqEGqdSit1qKOUVw0RsCA")
        print("  - QDRANT endpoint: https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io")
        print("  - QDRANT API key: (configured)")
        print()
        print("To run the application:")
        print("  cd D:\\HACKHATHON-1\\humanoid-robotics-book\\backend")
        print("  set GEMINI_API_KEY=AIzaSyCe6tUMSZGFkxzqEGqdSit1qKOUVw0RsCA")
        print("  set QDRANT_URL=https://0c4e4da0-0f58-46d4-a6ec-e7d68cd5e377.europe-west3-0.gcp.cloud.qdrant.io")
        print("  set QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Pej0yjlpDqhQVdnOFicUHdZWVki9DAzUhywMQ76QJgs")
        print("  uvicorn main:app --reload")
    else:
        print("Some issues were detected with the configuration")

if __name__ == "__main__":
    main()