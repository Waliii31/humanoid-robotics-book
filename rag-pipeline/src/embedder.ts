// src/embedder.ts
import { CohereClient } from 'cohere-ai';
import { Chunk, EmbeddingConfig } from './types';
import { RetryHandler, NetworkErrorHandler } from './utils';

export class Embedder {
  private client: CohereClient;

  constructor(private config: EmbeddingConfig) {
    this.client = new CohereClient({
      token: config.apiKey,
    });
  }

  async generateEmbeddings(chunks: Chunk[]): Promise<Chunk[]> {
    if (chunks.length === 0) {
      return chunks;
    }

    // Process in batches to respect API limits
    const batchSize = this.config.batchSize || 96; // Cohere's max batch size is 96
    const results: Chunk[] = [];

    for (let i = 0; i < chunks.length; i += batchSize) {
      const batch = chunks.slice(i, i + batchSize);

      console.log(`Generating embeddings for batch ${Math.floor(i / batchSize) + 1}/${Math.ceil(chunks.length / batchSize)}`);

      const response = await RetryHandler.executeWithRetry(async () => {
        return await this.client.embed({
          texts: batch.map(chunk => chunk.content),
          model: this.config.model || 'embed-multilingual-v3.0', // Using a reliable Cohere model
          inputType: 'search_document', // Optimal for document search
        });
      }, 3, 1000); // 3 retries with 1s base delay

      // Assign embeddings to chunks
      for (let j = 0; j < batch.length; j++) {
        const chunk = { ...batch[j] };
        chunk.embedding = response.embeddings[j] as number[];
        results.push(chunk);
      }
    }

    return results;
  }

  async generateQueryEmbedding(query: string): Promise<number[]> {
    const response = await RetryHandler.executeWithRetry(async () => {
      return await this.client.embed({
        texts: [query],
        model: this.config.model || 'embed-multilingual-v3.0',
        inputType: 'search_query', // Optimal for search queries
      });
    }, 3, 1000); // 3 retries with 1s base delay

    return response.embeddings[0] as number[];
  }
}