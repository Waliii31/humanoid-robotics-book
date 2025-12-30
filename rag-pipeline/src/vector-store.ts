// src/vector-store.ts
import { QdrantClient } from '@qdrant/js-client-rest';
import { Chunk, QdrantConfig } from './types';
import { RetryHandler } from './utils';

export class VectorStore {
  private client: QdrantClient;

  constructor(private config: QdrantConfig) {
    this.client = new QdrantClient({
      url: config.url,
      apiKey: config.apiKey,
    });
  }

  async ensureCollection(): Promise<void> {
    try {
      // Check if collection exists
      await RetryHandler.executeWithRetry(async () => {
        return await this.client.getCollection(this.config.collectionName);
      }, 3, 1000);
      console.log(`Collection ${this.config.collectionName} already exists`);
    } catch (error) {
      // Collection doesn't exist, create it
      console.log(`Creating collection ${this.config.collectionName}`);
      await RetryHandler.executeWithRetry(async () => {
        return await this.client.createCollection(this.config.collectionName, {
          vectors: {
            size: this.config.vectorSize,
            distance: 'Cosine', // Cosine distance is good for embeddings
          },
        });
      }, 3, 1000);
      console.log(`Collection ${this.config.collectionName} created successfully`);
    }
  }

  async storeChunks(chunks: Chunk[]): Promise<void> {
    if (chunks.length === 0) {
      console.log('No chunks to store');
      return;
    }

    console.log(`Storing ${chunks.length} chunks in Qdrant collection: ${this.config.collectionName}`);

    // Prepare points for Qdrant with comprehensive metadata
    const points = chunks.map(chunk => ({
      id: chunk.id,
      vector: chunk.embedding!,
      payload: {
        content: chunk.content,
        url: chunk.url,
        heading: chunk.heading,
        chunkIndex: chunk.chunkIndex,
        // Store additional metadata fields
        metadata: {
          ...chunk.metadata,
          createdAt: new Date().toISOString(),
          source: 'docusaurus-rag-pipeline'
        }
      }
    }));

    // Upsert points in batches to handle large datasets
    const batchSize = 100;
    for (let i = 0; i < points.length; i += batchSize) {
      const batch = points.slice(i, i + batchSize);

      await RetryHandler.executeWithRetry(async () => {
        return await this.client.upsert(this.config.collectionName, {
          points: batch
        });
      }, 3, 1000);

      console.log(`Stored batch ${Math.floor(i / batchSize) + 1}/${Math.ceil(points.length / batchSize)}`);
    }

    console.log(`Successfully stored ${chunks.length} chunks in Qdrant`);
  }

  async search(queryEmbedding: number[], limit: number = 5, filter?: Record<string, any>): Promise<Chunk[]> {
    // Prepare search parameters
    const searchParams: any = {
      vector: queryEmbedding,
      limit: limit,
      with_payload: true,
      with_vector: false, // Don't return vectors to save bandwidth
    };

    // Add filtering if provided
    if (filter && Object.keys(filter).length > 0) {
      searchParams.filter = this.buildFilter(filter);
    }

    const results = await RetryHandler.executeWithRetry(async () => {
      return await this.client.search(this.config.collectionName, searchParams);
    }, 3, 1000);

    return results.map(result => {
      const payload = result.payload;
      return {
        id: typeof result.id === 'string' ? result.id : result.id.toString(),
        content: payload?.content as string || '',
        url: payload?.url as string || '',
        heading: payload?.heading as string | undefined,
        chunkIndex: payload?.chunkIndex as number || 0,
        metadata: payload?.metadata || payload || {},
        embedding: undefined, // Already have the similarity score from search
        score: result.score
      };
    });
  }

  // Additional query methods
  async searchByUrl(url: string, queryEmbedding: number[], limit: number = 5): Promise<Chunk[]> {
    return this.search(queryEmbedding, limit, { url });
  }

  async searchByMetadata(metadataFilter: Record<string, any>, queryEmbedding: number[], limit: number = 5): Promise<Chunk[]> {
    return this.search(queryEmbedding, limit, metadataFilter);
  }

  async getChunkById(id: string): Promise<Chunk | null> {
    const records = await RetryHandler.executeWithRetry(async () => {
      return await this.client.retrieve(this.config.collectionName, {
        ids: [id],
        with_payload: true,
        with_vector: false,
      });
    }, 3, 1000);

    if (records.length === 0) {
      return null;
    }

    const record = records[0];
    const payload = record.payload;

    return {
      id: typeof record.id === 'string' ? record.id : record.id.toString(),
      content: payload?.content as string || '',
      url: payload?.url as string || '',
      heading: payload?.heading as string | undefined,
      chunkIndex: payload?.chunkIndex as number || 0,
      metadata: payload?.metadata || payload || {},
    };
  }

  // Helper method to build Qdrant filter
  private buildFilter(filter: Record<string, any>): any {
    const conditions: any[] = [];

    Object.entries(filter).forEach(([key, value]) => {
      if (typeof value === 'string') {
        conditions.push({
          key: `metadata.${key}`,
          match: { value: value }
        });
      } else if (typeof value === 'number') {
        conditions.push({
          key: `metadata.${key}`,
          range: { gte: value, lte: value }
        });
      } else {
        conditions.push({
          key: `metadata.${key}`,
          match: { value: JSON.stringify(value) }
        });
      }
    });

    return {
      must: conditions
    };
  }

  async deleteCollection(): Promise<void> {
    try {
      await RetryHandler.executeWithRetry(async () => {
        return await this.client.deleteCollection(this.config.collectionName);
      }, 3, 1000);
      console.log(`Collection ${this.config.collectionName} deleted successfully`);
    } catch (error) {
      console.error(`Error deleting collection ${this.config.collectionName}:`, error);
      throw error;
    }
  }

  async countVectors(): Promise<number> {
    const collectionInfo = await RetryHandler.executeWithRetry(async () => {
      return await this.client.getCollection(this.config.collectionName);
    }, 3, 1000);
    return collectionInfo.points_count;
  }
}