// src/validation-script.ts
import 'dotenv/config';
import { SemanticRetriever, RetrievalConfig } from './retriever';

async function runValidation() {
  console.log('Starting RAG Pipeline Validation...\n');

  // Configuration for validation
  const config: RetrievalConfig = {
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
  const requiredEnvVars = ['COHERE_API_KEY', 'QDRANT_URL'];
  const missingEnvVars = requiredEnvVars.filter(envVar => !process.env[envVar]);
  if (missingEnvVars.length > 0) {
    console.error(`Missing required environment variables: ${missingEnvVars.join(', ')}`);
    process.exit(1);
  }

  const retriever = new SemanticRetriever(config);

  // Run health check first
  console.log('1. Running health check...');
  const isHealthy = await retriever.healthCheck();
  console.log(`   Health check: ${isHealthy ? 'PASSED' : 'FAILED'}\n`);

  if (!isHealthy) {
    console.error('Pipeline is not healthy. Exiting validation.');
    process.exit(1);
  }

  // Get statistics
  console.log('2. Getting vector store statistics...');
  const stats = await retriever.getStats();
  console.log(`   Total vectors in store: ${stats.totalVectors}\n`);

  // Define validation queries based on typical book content
  console.log('3. Running retrieval quality validation...');

  const validationQueries = [
    {
      query: "What is physical AI?",
      expectedUrls: ["/intro", "/week01", "/week02"], // Placeholder URLs - these would match actual book content
      description: "Basic concept query"
    },
    {
      query: "humanoid robotics fundamentals",
      expectedUrls: ["/week11", "/week12"], // Placeholder URLs
      description: "Core topic query"
    },
    {
      query: "ROS2 basics",
      expectedUrls: ["/week03", "/week04", "/week05"], // Placeholder URLs
      description: "Technology-specific query"
    },
    {
      query: "simulation environments",
      expectedUrls: ["/week06", "/week07"], // Placeholder URLs
      description: "Methodology query"
    },
    {
      query: "NVIDIA Isaac tutorials",
      expectedUrls: ["/week08", "/week09", "/week10"], // Placeholder URLs
      description: "Platform-specific query"
    }
  ];

  // Run validation with the actual book content
  const validationData = validationQueries.map(vq => ({
    query: vq.query,
    expectedUrls: vq.expectedUrls,
    expectedContent: [] // We can add expected content snippets here if needed
  }));

  const validationResults = await retriever.validateRetrievalQuality(
    validationData,
    0.1 // Lower threshold for initial validation
  );

  console.log(`   Total queries tested: ${validationResults.totalQueries}`);
  console.log(`   Successful retrievals: ${validationResults.successfulRetrievals}`);
  console.log(`   Accuracy: ${(validationResults.accuracy * 100).toFixed(2)}%`);
  console.log('');

  // Detailed results
  console.log('4. Detailed validation results:');
  validationResults.detailedResults.forEach((result, index) => {
    const queryInfo = validationQueries[index];
    console.log(`   Query ${index + 1}: "${queryInfo?.query || result.query}" (${queryInfo?.description || 'No description'})`);
    console.log(`     Expected URLs: ${result.expectedUrls.join(', ')}`);
    console.log(`     Retrieved URLs: ${result.retrievedUrls.join(', ')}`);
    console.log(`     Scores: [${result.scores.map(s => s.toFixed(4)).join(', ')}]`);
    console.log(`     Result: ${result.success ? 'SUCCESS' : 'FAILURE'}`);
    console.log('');
  });

  // Performance test
  console.log('5. Running performance tests...');
  const perfResults = await retriever.measurePerformance("performance test", 3);
  console.log(`   Average latency: ${perfResults.avgLatency.toFixed(2)}ms`);
  console.log(`   Min latency: ${perfResults.minLatency}ms`);
  console.log(`   Max latency: ${perfResults.maxLatency}ms`);
  console.log(`   Successful retrievals: ${perfResults.successfulRetrievals}/3`);
  console.log('');

  // Sample retrieval test
  console.log('6. Sample retrieval test:');
  const sampleResults = await retriever.retrieve("humanoid robotics", { topK: 3 });

  if (sampleResults.length > 0) {
    console.log('   Top 3 results for "humanoid robotics":');
    sampleResults.forEach((result, index) => {
      console.log(`     ${index + 1}. [Score: ${(result.score || 0).toFixed(4)}] ${result.url}`);
      console.log(`        Content preview: ${result.content.substring(0, 100)}...`);
      console.log(`        Metadata keys: ${Object.keys(result.metadata).join(', ')}`);
      console.log('');
    });
  } else {
    console.log('   No results found for sample query');
  }

  // Final summary
  console.log('7. Validation Summary:');
  console.log(`   ✓ Health Check: ${isHealthy ? 'PASS' : 'FAIL'}`);
  console.log(`   ✓ Total Vectors: ${stats.totalVectors}`);
  console.log(`   ✓ Retrieval Accuracy: ${(validationResults.accuracy * 100).toFixed(2)}%`);
  console.log(`   ✓ Avg Latency: ${perfResults.avgLatency.toFixed(2)}ms`);
  console.log(`   ✓ Sample Retrieval: ${sampleResults.length > 0 ? 'PASS' : 'FAIL'}`);

  const overallPass = isHealthy && validationResults.accuracy > 0 && sampleResults.length > 0;
  console.log(`\n   OVERALL RESULT: ${overallPass ? 'PASS' : 'FAIL'}`);

  return {
    isHealthy,
    stats,
    validationResults,
    perfResults,
    sampleResults,
    overallPass
  };
}

// Execute if this is the main module
if (require.main === module) {
  runValidation()
    .then(results => {
      console.log('\nValidation completed successfully!');
      process.exit(results.overallPass ? 0 : 1);
    })
    .catch(error => {
      console.error('Validation failed with error:', error);
      process.exit(1);
    });
}

export { runValidation };