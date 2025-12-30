// src/pipeline.ts
import { WebCrawler } from './crawler';
import { DocumentChunker } from './chunker';
import { Embedder } from './embedder';
import { VectorStore } from './vector-store';
import { CrawlConfig, ChunkingConfig, EmbeddingConfig, QdrantConfig } from './types';

export interface PipelineConfig {
  crawl: CrawlConfig;
  chunking: ChunkingConfig;
  embedding: EmbeddingConfig;
  qdrant: QdrantConfig;
}

export class RAGPipeline {
  private crawler: WebCrawler;
  private chunker: DocumentChunker;
  private embedder: Embedder;
  private vectorStore: VectorStore;

  constructor(private config: PipelineConfig) {
    this.crawler = new WebCrawler(config.crawl);
    this.chunker = new DocumentChunker(config.chunking);
    this.embedder = new Embedder(config.embedding);
    this.vectorStore = new VectorStore(config.qdrant);
  }

  async run(): Promise<void> {
    console.log('Starting RAG pipeline...');

    // Step 1: Ensure Qdrant collection exists
    console.log('Setting up vector store...');
    await this.vectorStore.ensureCollection();

    // Step 2: Crawl the websites
    console.log('Crawling websites...');
    const crawledPages = await this.crawler.crawl();
    console.log(`Crawled ${crawledPages.length} pages`);

    // Step 3: Chunk the content
    console.log('Chunking content...');
    const chunks = this.chunker.chunkPages(crawledPages);
    console.log(`Created ${chunks.length} chunks`);

    // Step 4: Generate embeddings
    console.log('Generating embeddings...');
    const chunksWithEmbeddings = await this.embedder.generateEmbeddings(chunks);
    console.log('Embeddings generated successfully');

    // Step 5: Store in vector database
    console.log('Storing in vector database...');
    await this.vectorStore.storeChunks(chunksWithEmbeddings);
    console.log('Storage completed');

    console.log('RAG pipeline completed successfully!');
  }

  async search(query: string, limit: number = 5): Promise<any[]> {
    // Generate embedding for the query
    const queryEmbedding = await this.embedder.generateQueryEmbedding(query);

    // Search in vector store
    return await this.vectorStore.search(queryEmbedding, limit);
  }

  async getStats(): Promise<{ totalChunks: number }> {
    const totalChunks = await this.vectorStore.countVectors();
    return { totalChunks };
  }
}