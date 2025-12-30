// src/types.ts
export interface CrawlConfig {
  urls: string[];
  maxDepth?: number;
  rateLimitMs?: number;
  selectors?: {
    content: string;
    navLinks?: string;
    excludeSelectors?: string[];
  };
}

export interface CrawledPage {
  url: string;
  title: string;
  content: string;
  headings: Array<{
    level: number;
    text: string;
    id?: string;
  }>;
  metadata: Record<string, any>;
  createdAt: Date;
}

export interface Chunk {
  id: string;
  content: string;
  url: string;
  heading?: string;
  chunkIndex: number;
  embedding?: number[];
  metadata: Record<string, any>;
  score?: number;
}

export interface EmbeddingConfig {
  apiKey: string;
  model?: string;
  batchSize?: number;
}

export interface QdrantConfig {
  url: string;
  apiKey: string;
  collectionName: string;
  vectorSize: number;
}

export interface ChunkingConfig {
  chunkSize: number; // in characters
  overlapSize: number; // in characters
  delimiter?: string;
}