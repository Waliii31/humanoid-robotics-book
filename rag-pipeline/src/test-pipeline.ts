// src/test-pipeline.ts
import 'dotenv/config';
import { RAGPipeline, PipelineConfig } from './pipeline';

async function runTest() {
  console.log('Testing RAG Pipeline...');

  // Configuration for testing - using the local Docusaurus site
  const config: PipelineConfig = {
    crawl: {
      urls: ['http://localhost:3000'], // This would be the local Docusaurus site
      maxDepth: 2,
      rateLimitMs: 1000,
      selectors: {
        content: 'article, .markdown, .docItemContainer, main',
        excludeSelectors: ['.pagination-nav', '.theme-admonition', 'footer']
      }
    },
    chunking: {
      chunkSize: 512, // characters
      overlapSize: 50, // characters
    },
    embedding: {
      apiKey: process.env.COHERE_API_KEY || 'YOUR_COHERE_API_KEY',
      model: 'embed-multilingual-v3.0',
      batchSize: 32
    },
    qdrant: {
      url: process.env.QDRANT_URL || 'http://localhost:6333',
      apiKey: process.env.QDRANT_API_KEY || '',
      collectionName: 'docusaurus_docs',
      vectorSize: 1024 // Cohere's embedding size for multilingual model
    }
  };

  // Check if required environment variables are set
  if (!process.env.COHERE_API_KEY) {
    console.error('COHERE_API_KEY environment variable is required');
    process.exit(1);
  }

  if (!process.env.QDRANT_URL) {
    console.error('QDRANT_URL environment variable is required');
    process.exit(1);
  }

  try {
    const pipeline = new RAGPipeline(config);

    // Run the full pipeline
    await pipeline.run();

    // Get stats
    const stats = await pipeline.getStats();
    console.log(`Total chunks in database: ${stats.totalChunks}`);

    // Test search functionality
    console.log('Testing search functionality...');
    const searchResults = await pipeline.search('What is physical AI?');

    if (searchResults.length > 0) {
      console.log('Search results:');
      searchResults.forEach((result, index) => {
        console.log(`${index + 1}. Score: ${result.score}, URL: ${result.url}`);
        console.log(`   Content preview: ${result.content.substring(0, 100)}...`);
        console.log('---');
      });
    } else {
      console.log('No search results found');
    }

    console.log('Pipeline test completed successfully!');
  } catch (error) {
    console.error('Pipeline test failed:', error);
    process.exit(1);
  }
}

// For testing with the actual deployed site instead of localhost
async function runTestWithDeployedSite() {
  console.log('Testing RAG Pipeline with deployed site...');

  // Configuration for testing - using the actual deployed Docusaurus site
  // For this repo, we'll use a placeholder - in real usage, this would be the actual deployed URL
  const deployedSiteUrl = 'https://your-docusaurus-site.example.com'; // Placeholder

  // If we want to test with the current repo's content, we'd need to build and serve it first
  // For now, let's create a more comprehensive test that shows how it would work
  console.log(`Testing with site: ${deployedSiteUrl}`);
  console.log('Note: For actual testing, replace the URL with your deployed Docusaurus site');

  // Mock test showing the expected workflow
  console.log('\nExpected workflow:');
  console.log('1. Crawler would visit: ' + deployedSiteUrl);
  console.log('2. Extract content from Docusaurus documentation pages');
  console.log('3. Chunk content into semantic pieces');
  console.log('4. Generate embeddings using Cohere');
  console.log('5. Store in Qdrant vector database');
  console.log('6. Enable semantic search functionality');

  // Example of how search would work
  const exampleQuery = 'humanoid robotics fundamentals';
  console.log(`\nExample search query: "${exampleQuery}"`);
  console.log('This would return semantically similar content from the documentation');
}

// Run the appropriate test
if (require.main === module) {
  // Check if we're trying to test with localhost (requires local server)
  const testWithLocalhost = process.argv.includes('--localhost');

  if (testWithLocalhost) {
    runTest().catch(console.error);
  } else {
    runTestWithDeployedSite();
  }
}