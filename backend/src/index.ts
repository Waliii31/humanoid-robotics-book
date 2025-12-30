// src/index.ts - Main entry point for the backend RAG pipeline
import 'dotenv/config';
import { RAGPipeline, PipelineConfig } from './pipeline';

async function main() {
  console.log('Starting Docusaurus RAG Backend Pipeline...');

  // Configuration - these should be set via environment variables
  const config: PipelineConfig = {
    crawl: {
      urls: [process.env.DEPLOYED_SITE_URL || 'https://your-docusaurus-site.example.com'],
      maxDepth: parseInt(process.env.CRAWL_MAX_DEPTH || '2'),
      rateLimitMs: parseInt(process.env.CRAWL_RATE_LIMIT || '1000'),
      selectors: {
        content: process.env.CONTENT_SELECTOR || 'article, .markdown, .docItemContainer, main',
        excludeSelectors: process.env.EXCLUDE_SELECTORS?.split(',') || ['.pagination-nav', '.theme-admonition', 'footer']
      }
    },
    chunking: {
      chunkSize: parseInt(process.env.CHUNK_SIZE || '512'),
      overlapSize: parseInt(process.env.OVERLAP_SIZE || '50'),
    },
    embedding: {
      apiKey: process.env.COHERE_API_KEY || '',
      model: process.env.COHERE_MODEL || 'embed-multilingual-v3.0',
      batchSize: parseInt(process.env.EMBEDDING_BATCH_SIZE || '32')
    },
    qdrant: {
      url: process.env.QDRANT_URL || '',
      apiKey: process.env.QDRANT_API_KEY || '',
      collectionName: process.env.QDRANT_COLLECTION_NAME || 'docusaurus_docs',
      vectorSize: parseInt(process.env.VECTOR_SIZE || '1024')
    }
  };

  // Validate required configuration
  const requiredEnvVars = [
    'COHERE_API_KEY',
    'QDRANT_URL',
    'DEPLOYED_SITE_URL'
  ];

  const missingEnvVars = requiredEnvVars.filter(envVar => !process.env[envVar]);
  if (missingEnvVars.length > 0) {
    console.error(`Missing required environment variables: ${missingEnvVars.join(', ')}`);
    console.log('\nCreate a .env file with the following variables:');
    console.log('DEPLOYED_SITE_URL=https://your-docusaurus-site.com');
    console.log('COHERE_API_KEY=your_cohere_api_key');
    console.log('QDRANT_URL=your_qdrant_cluster_url');
    console.log('QDRANT_API_KEY=your_qdrant_api_key (if required)');
    process.exit(1);
  }

  try {
    const pipeline = new RAGPipeline(config);

    // Run the pipeline
    await pipeline.run();

    // Show final statistics
    const stats = await pipeline.getStats();
    console.log(`\nPipeline completed successfully!`);
    console.log(`Total documents indexed: ${stats.totalChunks}`);

  } catch (error) {
    console.error('Pipeline failed:', error);
    process.exit(1);
  }
}

// Execute if this is the main module
if (require.main === module) {
  main().catch(error => {
    console.error('Unhandled error:', error);
    process.exit(1);
  });
}