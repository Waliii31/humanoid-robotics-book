// src/vector-store.ts
import { QdrantClient } from '@qdrant/js-client-rest';
import { Chunk, QdrantConfig } from './types';

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
      await this.client.getCollection(this.config.collectionName);
      console.log(`Collection ${this.config.collectionName} already exists`);
    } catch (error) {
      // Collection doesn't exist, create it
      console.log(`Creating collection ${this.config.collectionName}`);
      await this.client.createCollection(this.config.collectionName, {
        vectors: {
          size: this.config.vectorSize,
          distance: 'Cosine', // Cosine distance is good for embeddings
        },
      });
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
          source: 'docusaurus-rag-backend'
        }
      }
    }));

    // Upsert points in batches to handle large datasets
    const batchSize = 100;
    for (let i = 0; i < points.length; i += batchSize) {
      const batch = points.slice(i, i + batchSize);
      await this.client.upsert(this.config.collectionName, {
        points: batch
      });
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

    const results = await this.client.search(this.config.collectionName, searchParams);

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

  async searchByUrls(urls: string[], queryEmbedding: number[], limit: number = 5): Promise<Chunk[]> {
    // Qdrant doesn't directly support "in" queries, so we need to search for each URL and merge results
    // For now, we'll do multiple searches and combine results, sorting by score
    const allResults: Chunk[] = [];

    for (const url of urls) {
      const results = await this.search(queryEmbedding, limit, { url });
      allResults.push(...results);
    }

    // Sort by score and return top results
    return allResults
      .sort((a, b) => (b.score || 0) - (a.score || 0))
      .slice(0, limit);
  }

  async searchByMetadata(metadataFilter: Record<string, any>, queryEmbedding: number[], limit: number = 5): Promise<Chunk[]> {
    return this.search(queryEmbedding, limit, metadataFilter);
  }

  async getChunkById(id: string): Promise<Chunk | null> {
    const records = await this.client.retrieve(this.config.collectionName, {
      ids: [id],
      with_payload: true,
      with_vector: false,
    });

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
      await this.client.deleteCollection(this.config.collectionName);
      console.log(`Collection ${this.config.collectionName} deleted successfully`);
    } catch (error) {
      console.error(`Error deleting collection ${this.config.collectionName}:`, error);
      throw error;
    }
  }

  async countVectors(): Promise<number> {
    const collectionInfo = await this.client.getCollection(this.config.collectionName);
    return collectionInfo.points_count;
  }
}