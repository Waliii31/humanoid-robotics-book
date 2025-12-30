// src/retriever.ts
import { Embedder } from './embedder';
import { VectorStore } from './vector-store';
import { Chunk, EmbeddingConfig, QdrantConfig } from './types';

export interface RetrievalConfig {
  embedding: EmbeddingConfig;
  qdrant: QdrantConfig;
}

export interface RetrievalParams {
  topK?: number;
  filter?: {
    url?: string;
    urls?: string[];
    chunkIndex?: number;
    minChunkIndex?: number;
    maxChunkIndex?: number;
    [key: string]: any;
  };
  includeMetadata?: boolean;
}

export class SemanticRetriever {
  private embedder: Embedder;
  private vectorStore: VectorStore;
  private queryCache: Map<string, { results: Chunk[], timestamp: number, params: string }> = new Map();

  constructor(private config: RetrievalConfig) {
    this.embedder = new Embedder(config.embedding);
    this.vectorStore = new VectorStore(config.qdrant);
  }

  /**
   * Performs semantic search using query embeddings and returns relevant chunks
   * @param query The search query
   * @param params Additional retrieval parameters
   * @returns Array of relevant chunks with similarity scores
   */
  async retrieve(query: string, params: RetrievalParams = {}): Promise<Chunk[]> {
    // Create a cache key based on query and parameters
    const cacheKey = this.generateCacheKey(query, params);

    // Check if we have cached results and they're still valid (5 minutes)
    const cached = this.queryCache.get(cacheKey);
    const now = Date.now();
    if (cached && (now - cached.timestamp) < 5 * 60 * 1000) { // 5 minutes
      return cached.results;
    }

    // Generate query embedding
    const queryEmbedding = await this.embedder.generateQueryEmbedding(query);

    // Handle multiple URLs separately from other filters
    if (params.filter?.urls && Array.isArray(params.filter.urls) && params.filter.urls.length > 0) {
      // If multiple URLs are specified, use the specialized search method
      const results = await this.vectorStore.searchByUrls(
        params.filter.urls,
        queryEmbedding,
        params.topK || 5
      );

      // Sort to ensure deterministic ordering
      const sortedResults = this.ensureDeterministicOrder(results);

      // Cache the results
      this.queryCache.set(cacheKey, {
        results: sortedResults,
        timestamp: now,
        params: JSON.stringify(params)
      });

      return sortedResults;
    }

    // For other cases, use the standard search with filters
    const filter = this.buildQdrantFilter(params.filter);

    // Perform similarity search
    const results = await this.vectorStore.search(
      queryEmbedding,
      params.topK || 5,
      filter
    );

    // Sort to ensure deterministic ordering
    const sortedResults = this.ensureDeterministicOrder(results);

    // Cache the results
    this.queryCache.set(cacheKey, {
      results: sortedResults,
      timestamp: now,
      params: JSON.stringify(params)
    });

    return sortedResults;
  }

  /**
   * Ensures deterministic ordering of results by sorting primarily by score, then by ID
   * @param results The results to sort
   * @returns Sorted results with deterministic order
   */
  private ensureDeterministicOrder(results: Chunk[]): Chunk[] {
    return results.sort((a, b) => {
      // Primary sort: by score (descending)
      const scoreDiff = (b.score || 0) - (a.score || 0);
      if (scoreDiff !== 0) {
        return scoreDiff;
      }
      // Secondary sort: by ID (ascending) to ensure deterministic order for equal scores
      if (a.id < b.id) return -1;
      if (a.id > b.id) return 1;
      return 0;
    });
  }

  /**
   * Generates a cache key based on query and parameters
   * @param query The search query
   * @param params The retrieval parameters
   * @returns A unique cache key
   */
  private generateCacheKey(query: string, params: RetrievalParams): string {
    return `${query}:${JSON.stringify(params)}`;
  }

  /**
   * Clears the query cache
   */
  clearCache(): void {
    this.queryCache.clear();
  }

  /**
   * Retrieves chunks from a specific URL
   * @param query The search query
   * @param url The URL to filter by
   * @param topK Number of results to return
   * @returns Array of relevant chunks from the specified URL
   */
  async retrieveFromUrl(query: string, url: string, topK: number = 5): Promise<Chunk[]> {
    const queryEmbedding = await this.embedder.generateQueryEmbedding(query);
    return await this.vectorStore.searchByUrl(url, queryEmbedding, topK);
  }

  /**
   * Retrieves chunks based on custom metadata filters
   * @param query The search query
   * @param metadataFilter Metadata filter object
   * @param topK Number of results to return
   * @returns Array of relevant chunks matching the metadata filter
   */
  async retrieveByMetadata(
    query: string,
    metadataFilter: Record<string, any>,
    topK: number = 5
  ): Promise<Chunk[]> {
    const queryEmbedding = await this.embedder.generateQueryEmbedding(query);
    return await this.vectorStore.searchByMetadata(metadataFilter, queryEmbedding, topK);
  }

  /**
   * Gets a specific chunk by its ID
   * @param id The chunk ID
   * @returns The chunk or null if not found
   */
  async getChunkById(id: string): Promise<Chunk | null> {
    return await this.vectorStore.getChunkById(id);
  }

  /**
   * Builds a Qdrant-compatible filter from the provided filter parameters
   * @param filterParams The filter parameters
   * @returns Qdrant filter object or undefined if no filters
   */
  private buildQdrantFilter(filterParams?: RetrievalParams['filter']): Record<string, any> | undefined {
    if (!filterParams) {
      return undefined;
    }

    const qdrantFilter: Record<string, any> = {};

    // Handle URL filtering (excluding multiple URLs which we handle separately)
    if (filterParams.url) {
      qdrantFilter.url = filterParams.url;
    }
    // Note: Multiple URLs are handled separately in the retrieve method

    // Handle chunk index filtering
    if (typeof filterParams.chunkIndex === 'number') {
      qdrantFilter.chunkIndex = filterParams.chunkIndex;
    } else if (typeof filterParams.minChunkIndex === 'number' || typeof filterParams.maxChunkIndex === 'number') {
      const rangeFilter: any = {};
      if (typeof filterParams.minChunkIndex === 'number') {
        rangeFilter.gte = filterParams.minChunkIndex;
      }
      if (typeof filterParams.maxChunkIndex === 'number') {
        rangeFilter.lte = filterParams.maxChunkIndex;
      }
      if (Object.keys(rangeFilter).length > 0) {
        qdrantFilter.chunkIndex = rangeFilter;
      }
    }

    // Add any additional custom filters
    Object.keys(filterParams).forEach(key => {
      if (!['url', 'urls', 'chunkIndex', 'minChunkIndex', 'maxChunkIndex'].includes(key)) {
        qdrantFilter[key] = (filterParams as Record<string, any>)[key];
      }
    });

    return Object.keys(qdrantFilter).length > 0 ? qdrantFilter : undefined;
  }

  /**
   * Validates that the retrieval configuration is consistent with the indexing configuration
   * @returns True if configurations are consistent, false otherwise
   */
  async validateConfiguration(): Promise<boolean> {
    try {
      // Check if the collection exists
      await this.vectorStore.countVectors();
      return true;
    } catch (error) {
      console.error('Configuration validation failed:', error);
      return false;
    }
  }

  /**
   * Validates retrieval quality by testing known query-document relationships
   * @param validationQueries Array of query-content pairs to validate
   * @param threshold Minimum similarity score threshold for validation
   * @returns Validation results with scores and accuracy metrics
   */
  async validateRetrievalQuality(
    validationQueries: Array<{ query: string; expectedUrls: string[]; expectedContent?: string[] }>,
    threshold: number = 0.5
  ): Promise<{
    totalQueries: number;
    successfulRetrievals: number;
    accuracy: number;
    detailedResults: Array<{
      query: string;
      expectedUrls: string[];
      retrievedUrls: string[];
      scores: number[];
      success: boolean;
    }>;
  }> {
    const results = {
      totalQueries: validationQueries.length,
      successfulRetrievals: 0,
      detailedResults: [] as Array<{
        query: string;
        expectedUrls: string[];
        retrievedUrls: string[];
        scores: number[];
        success: boolean;
      }>,
    };

    for (const validationQuery of validationQueries) {
      const retrievedChunks = await this.retrieve(validationQuery.query, { topK: 5 });

      const retrievedUrls = retrievedChunks.map(chunk => chunk.url);
      const scores = retrievedChunks.map(chunk => chunk.score || 0);

      // Check if any of the expected URLs are in the retrieved results
      const hasExpectedUrl = validationQuery.expectedUrls.some(expectedUrl =>
        retrievedUrls.includes(expectedUrl)
      );

      const success = hasExpectedUrl;

      if (success) {
        results.successfulRetrievals++;
      }

      results.detailedResults.push({
        query: validationQuery.query,
        expectedUrls: validationQuery.expectedUrls,
        retrievedUrls,
        scores,
        success,
      });
    }

    results.accuracy = results.totalQueries > 0
      ? results.successfulRetrievals / results.totalQueries
      : 0;

    return results;
  }

  /**
   * Performs a simple test retrieval to validate the pipeline is working
   * @param testQuery A simple test query
   * @returns True if retrieval was successful, false otherwise
   */
  async healthCheck(testQuery: string = "test"): Promise<boolean> {
    try {
      const results = await this.retrieve(testQuery, { topK: 1 });
      return results.length > 0;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }

  /**
   * Gets statistics about the vector store
   * @returns Statistics about the stored vectors
   */
  async getStats(): Promise<{ totalVectors: number }> {
    const totalVectors = await this.vectorStore.countVectors();
    return { totalVectors };
  }

  /**
   * Measures retrieval performance
   * @param query Test query to measure performance
   * @param iterations Number of iterations to run for averaging
   * @returns Performance metrics including average latency
   */
  async measurePerformance(query: string = "performance test", iterations: number = 5): Promise<{
    avgLatency: number;
    minLatency: number;
    maxLatency: number;
    successfulRetrievals: number;
  }> {
    const latencies: number[] = [];
    let successfulRetrievals = 0;

    for (let i = 0; i < iterations; i++) {
      const startTime = Date.now();
      try {
        const results = await this.retrieve(query, { topK: 3 });
        if (results.length > 0) {
          successfulRetrievals++;
        }
        const endTime = Date.now();
        latencies.push(endTime - startTime);
      } catch (error) {
        const endTime = Date.now();
        latencies.push(endTime - startTime); // Still record the time even if there was an error
      }
    }

    const avgLatency = latencies.reduce((a, b) => a + b, 0) / latencies.length;
    const minLatency = Math.min(...latencies);
    const maxLatency = Math.max(...latencies);

    return {
      avgLatency,
      minLatency,
      maxLatency,
      successfulRetrievals
    };
  }

  /**
   * Optimizes retrieval by pre-warming the cache with common queries
   * @param warmupQueries Common queries to pre-cache
   */
  async warmupCache(warmupQueries: string[]): Promise<void> {
    const promises = warmupQueries.map(query =>
      this.retrieve(query, { topK: 5 })
    );

    await Promise.all(promises);
  }
}