# Docusaurus RAG Pipeline

A comprehensive pipeline for indexing Docusaurus documentation sites, generating semantic embeddings, and storing them in a vector database for RAG chatbot applications.

## Features

- **Web Crawling**: Efficiently crawls Docusaurus documentation sites
- **Content Extraction**: Extracts clean text content while preserving document hierarchy
- **Document Chunking**: Configurable chunking strategy with overlap support
- **Semantic Embeddings**: Uses Cohere's state-of-the-art embedding models
- **Vector Storage**: Stores embeddings in Qdrant vector database with rich metadata
- **Semantic Search**: Enables semantic similarity search for RAG applications
- **Error Handling**: Robust retry mechanisms and error handling
- **Configurable**: Highly configurable through environment variables

## Prerequisites

- Node.js 18+
- Cohere API key
- Qdrant Cloud account (or self-hosted Qdrant instance)

## Installation

```bash
cd rag-pipeline
npm install
```

## Configuration

Create a `.env` file in the `rag-pipeline` directory with the following variables:

```env
# Cohere Configuration
COHERE_API_KEY=your-cohere-api-key-here
COHERE_MODEL=embed-multilingual-v3.0

# Qdrant Configuration
QDRANT_URL=https://your-cluster-url.qdrant.tech:6333
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=docusaurus_docs

# Crawling Configuration
DEPLOYED_SITE_URL=https://your-docusaurus-site.com
CRAWL_MAX_DEPTH=2
CRAWL_RATE_LIMIT=1000

# Chunking Configuration
CHUNK_SIZE=512
OVERLAP_SIZE=50
EMBEDDING_BATCH_SIZE=32

# Content Selection (optional)
CONTENT_SELECTOR=article, .markdown, .docItemContainer, main
EXCLUDE_SELECTORS=.pagination-nav,.theme-admonition,footer
VECTOR_SIZE=1024
```

## Usage

### Run the Full Pipeline

```bash
npm start
```

This will:
1. Crawl the specified Docusaurus site
2. Extract and chunk the content
3. Generate semantic embeddings
4. Store the embeddings in Qdrant

### Run in Development Mode

```bash
npm run dev
```

### Build the Project

```bash
npm run build
```

## Programmatic Usage

You can also use the pipeline programmatically:

```typescript
import { RAGPipeline, PipelineConfig } from './pipeline';

const config: PipelineConfig = {
  crawl: {
    urls: ['https://your-docusaurus-site.com'],
    maxDepth: 2,
    rateLimitMs: 1000,
    selectors: {
      content: 'article, .markdown, .docItemContainer, main',
      excludeSelectors: ['.pagination-nav', '.theme-admonition', 'footer']
    }
  },
  chunking: {
    chunkSize: 512,
    overlapSize: 50,
  },
  embedding: {
    apiKey: 'your-cohere-api-key',
    model: 'embed-multilingual-v3.0',
    batchSize: 32
  },
  qdrant: {
    url: 'https://your-cluster.qdrant.tech:6333',
    apiKey: 'your-qdrant-api-key',
    collectionName: 'docusaurus_docs',
    vectorSize: 1024
  }
};

const pipeline = new RAGPipeline(config);
await pipeline.run();

// Perform semantic search
const results = await pipeline.search('your search query');
```

## Pipeline Components

### 1. Web Crawler
- Discovers and extracts content from Docusaurus sites
- Respects robots.txt and rate limiting
- Handles navigation through documentation structure

### 2. Document Chunker
- Splits content into semantically coherent chunks
- Configurable chunk size and overlap
- Preserves document context and metadata

### 3. Embedder
- Uses Cohere's embedding API for semantic vectors
- Batch processing for efficiency
- Retry mechanisms for API reliability

### 4. Vector Store
- Stores embeddings in Qdrant vector database
- Rich metadata storage (URL, headings, etc.)
- Efficient similarity search

## API

### RAGPipeline Methods

- `run()`: Execute the full pipeline
- `search(query: string, limit?: number)`: Perform semantic search
- `getStats()`: Get pipeline statistics

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere API key | required |
| `QDRANT_URL` | Qdrant cluster URL | required |
| `QDRANT_API_KEY` | Qdrant API key | required |
| `DEPLOYED_SITE_URL` | URL of Docusaurus site to crawl | required |
| `CRAWL_MAX_DEPTH` | Maximum depth for crawling | 2 |
| `CRAWL_RATE_LIMIT` | Rate limit between requests (ms) | 1000 |
| `CHUNK_SIZE` | Size of content chunks | 512 |
| `OVERLAP_SIZE` | Overlap between chunks | 50 |
| `EMBEDDING_BATCH_SIZE` | Batch size for embedding API | 32 |
| `QDRANT_COLLECTION_NAME` | Qdrant collection name | docusaurus_docs |
| `VECTOR_SIZE` | Embedding vector size | 1024 |

## Testing

To run tests:

```bash
npm test
```

## Architecture

The pipeline follows a modular architecture:

```
[URL List] → [Crawler] → [Chunker] → [Embedder] → [Vector Store]
```

Each component is independently configurable and testable.

## Troubleshooting

1. **Cohere API errors**: Ensure your API key is valid and you have sufficient credits
2. **Qdrant connection errors**: Verify your cluster URL and API key
3. **Crawling issues**: Check that the target site is accessible and not blocking automated requests
4. **Rate limiting**: Adjust the `CRAWL_RATE_LIMIT` value as needed

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

MIT