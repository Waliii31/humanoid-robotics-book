// Ingest local markdown files into Qdrant
import 'dotenv/config';
import * as fs from 'fs';
import * as path from 'path';
import { QdrantClient } from '@qdrant/js-client-rest';
import { Embedder } from './src/embedder';
import { DocumentChunker } from './src/chunker';

// Define PointStruct type for Qdrant upsert
interface PointStruct {
  id: number | string;
  vector: number[];
  payload?: Record<string, unknown>;
}

interface Document {
  id: string;
  title: string;
  content: string;
  filePath: string;
  url: string;
}

class LocalMarkdownIngester {
  private qdrantClient: QdrantClient;
  private embedder: Embedder;
  private chunker: DocumentChunker;

  constructor(
    qdrantUrl: string,
    qdrantApiKey: string,
    cohereApiKey: string
  ) {
    this.qdrantClient = new QdrantClient({
      url: qdrantUrl,
      apiKey: qdrantApiKey,
    });

    this.embedder = new Embedder({
      apiKey: cohereApiKey,
      model: 'embed-multilingual-v3.0',
      batchSize: 32,
    });

    this.chunker = new DocumentChunker({
      chunkSize: 512,
      overlapSize: 50,
    });
  }

  async readMarkdownFiles(docsPath: string): Promise<Document[]> {
    const documents: Document[] = [];
    let docId = 0;

    const walkDir = (dir: string, baseDir: string) => {
      const files = fs.readdirSync(dir);

      for (const file of files) {
        const filePath = path.join(dir, file);
        const stat = fs.statSync(filePath);

        if (stat.isDirectory()) {
          // Skip node_modules and other irrelevant directories
          if (!file.startsWith('.') && file !== 'node_modules') {
            walkDir(filePath, baseDir);
          }
        } else if (file.endsWith('.md')) {
          try {
            const content = fs.readFileSync(filePath, 'utf-8');
            const relativePath = path.relative(baseDir, filePath);
            const urlPath = relativePath.replace(/\\/g, '/').replace(/\.md$/, '');

            // Extract title from frontmatter or filename
            const titleMatch = content.match(/^---[\s\S]*?title:\s*(.+?)(?:\n|$)/);
            const title = titleMatch ? titleMatch[1].trim() : file.replace('.md', '');

            documents.push({
              id: `doc_${docId++}`,
              title,
              content,
              filePath: relativePath,
              url: `/docs/${urlPath}`,
            });

            console.log(`✓ Loaded: ${relativePath}`);
          } catch (error) {
            console.error(`✗ Failed to read ${filePath}:`, error);
          }
        }
      }
    };

    walkDir(docsPath, docsPath);
    return documents;
  }

  async ensureCollection(collectionName: string, vectorSize: number = 768): Promise<void> {
    try {
      const col = await this.qdrantClient.getCollection(collectionName);

      // Robustly extract vector size from collection config
      let existingSize: number | undefined;
      const vectorsConfig = col?.config?.params?.vectors as Record<string, unknown> | undefined;
      if (vectorsConfig) {
        if (typeof (vectorsConfig as { size?: number }).size === 'number') {
          existingSize = (vectorsConfig as { size: number }).size;
        } else if (typeof vectorsConfig === 'object') {
          // Named vectors: take the first vector config
          const firstKey = Object.keys(vectorsConfig)[0];
          const firstVec = vectorsConfig[firstKey] as { size?: number } | undefined;
          if (firstKey && firstVec?.size) {
            existingSize = firstVec.size;
          }
        }
      }

      if (!existingSize || existingSize !== vectorSize) {
        console.warn(`⚠️  Collection "${collectionName}" exists with vector size ${existingSize} but expected ${vectorSize}. Recreating collection to match embedding dimension.`);
        await this.qdrantClient.recreateCollection(collectionName, {
          vectors: {
            size: vectorSize,
            distance: 'Cosine',
          },
        });
        console.log(`✓ Collection "${collectionName}" recreated with vector size ${vectorSize}`);
      } else {
        console.log(`✓ Collection "${collectionName}" already exists with correct vector size (${existingSize})`);
      }
    } catch (err) {
      console.log(`Creating collection "${collectionName}"...`);
      await this.qdrantClient.recreateCollection(collectionName, {
        vectors: {
          size: vectorSize,
          distance: 'Cosine',
        },
      });
      console.log(`✓ Collection "${collectionName}" created`);
    }
  }

  // Detect embedding vector size by asking the embedder to create a single query embedding
  // This lets us create the Qdrant collection with the correct `size` to avoid dimension errors.
  async detectEmbeddingSize(): Promise<number> {
    try {
      const testEmbedding = await this.embedder.generateQueryEmbedding('test');
      if (Array.isArray(testEmbedding) && testEmbedding.length > 0) {
        return testEmbedding.length;
      }
    } catch (e) {
      // Fall through to default
    }

    return 768; // sensible default
  }

  async ingestDocuments(documents: Document[], collectionName: string): Promise<void> {
    console.log(`\n📚 Processing ${documents.length} documents...\n`);

    let totalChunks = 0;
    const points: PointStruct[] = [];
    let pointId = 0;

    for (const doc of documents) {
      // Chunk the document
      const chunks = this.chunker.chunkPages([
        {
          url: doc.url,
          title: doc.title,
          content: doc.content,
          headings: [],
          metadata: {
            source: doc.filePath,
            title: doc.title,
          },
          createdAt: new Date(),
        },
      ]);

      console.log(`  ${doc.title}: ${chunks.length} chunks`);

      // Generate embeddings for chunks
      const chunksWithEmbeddings = await this.embedder.generateEmbeddings(chunks);

      // Prepare points for Qdrant
      for (const chunk of chunksWithEmbeddings) {
        if (chunk.embedding && chunk.embedding.length > 0) {
          points.push({
            id: pointId++,
            vector: chunk.embedding,
            payload: {
              document_id: doc.id,
              title: doc.title,
              content: chunk.content,
              source: doc.filePath,
              url: doc.url,
              chunk_index: chunk.chunkIndex || 0,
            },
          });
        }
      }

      totalChunks += chunks.length;
    }

    // Upload all points to Qdrant
    if (points.length > 0) {
      console.log(`\n📤 Uploading ${points.length} chunks to Qdrant...`);

      // Upload in batches to avoid timeout
      const batchSize = 100;
      for (let i = 0; i < points.length; i += batchSize) {
        const batch = points.slice(i, Math.min(i + batchSize, points.length));
        await this.qdrantClient.upsert(collectionName, {
          points: batch,
        });
        console.log(`  ✓ Uploaded ${Math.min(i + batchSize, points.length)}/${points.length}`);
      }

      console.log(`\n✅ Successfully indexed ${totalChunks} chunks into Qdrant!`);
    } else {
      console.log('⚠️  No chunks generated with embeddings');
    }
  }
}

async function main() {
  const qdrantUrl = process.env.QDRANT_URL;
  const qdrantApiKey = process.env.QDRANT_API_KEY || '';
  const cohereApiKey = process.env.COHERE_API_KEY;
  const collectionName = process.env.QDRANT_COLLECTION_NAME || 'docusaurus_docs';
  const docsPath = process.env.DOCS_PATH || '../docs';

  // Validate env vars
  if (!qdrantUrl) {
    console.error('❌ QDRANT_URL environment variable is not set');
    console.log('\nPlease configure your .env file with:');
    console.log('QDRANT_URL=https://your-qdrant-url.cloud.qdrant.io');
    console.log('QDRANT_API_KEY=your_api_key');
    console.log('COHERE_API_KEY=your_cohere_key');
    process.exit(1);
  }

  if (!cohereApiKey) {
    console.error('❌ COHERE_API_KEY environment variable is not set');
    process.exit(1);
  }

  try {
    console.log('🚀 Starting Local Markdown Ingestion...\n');

    const ingester = new LocalMarkdownIngester(qdrantUrl, qdrantApiKey, cohereApiKey);

    // Detect embedding dimension and ensure collection exists with matching vector size
    const detectedDim = await ingester.detectEmbeddingSize();
    console.log(`Detected embedding dimension: ${detectedDim}`);
    await ingester.ensureCollection(collectionName, detectedDim);

    // Read local markdown files
    console.log(`\n📖 Reading markdown files from: ${docsPath}\n`);
    const documents = await ingester.readMarkdownFiles(docsPath);

    if (documents.length === 0) {
      console.error('❌ No markdown files found in docs directory');
      process.exit(1);
    }

    console.log(`\n✓ Found ${documents.length} documents\n`);

    // Ingest documents
    await ingester.ingestDocuments(documents, collectionName);

    console.log('\n✨ Ingestion complete!');
  } catch (error) {
    console.error('❌ Ingestion failed:', error);
    process.exit(1);
  }
}

main();
