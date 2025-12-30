// src/chunker.ts
import { Chunk, ChunkingConfig, CrawledPage } from './types';
import { v4 as uuidv4 } from 'uuid';

export class DocumentChunker {
  constructor(private config: ChunkingConfig) {}

  chunkPages(pages: CrawledPage[]): Chunk[] {
    const chunks: Chunk[] = [];

    pages.forEach(page => {
      const pageChunks = this.chunkPage(page);
      chunks.push(...pageChunks);
    });

    return chunks;
  }

  private chunkPage(page: CrawledPage): Chunk[] {
    const chunks: Chunk[] = [];

    // Clean up the content
    const cleanContent = this.cleanContent(page.content);

    // Split content into chunks
    const contentChunks = this.splitContent(cleanContent);

    // Process each heading to associate with relevant chunks
    const headingMap = this.createHeadingMap(page.headings);

    contentChunks.forEach((chunkText, index) => {
      // Find the most relevant heading for this chunk
      const relevantHeading = this.findRelevantHeading(chunkText, headingMap, page.url);

      chunks.push({
        id: uuidv4(),
        content: chunkText,
        url: page.url,
        heading: relevantHeading,
        chunkIndex: index,
        metadata: {
          ...page.metadata,
          title: page.title,
          source: page.url,
          chunkCount: contentChunks.length,
          chunkIndex: index
        }
      });
    });

    return chunks;
  }

  private cleanContent(content: string): string {
    // Remove extra whitespace and normalize
    return content
      .replace(/\n+/g, ' ')
      .replace(/\s+/g, ' ')
      .trim();
  }

  private splitContent(content: string): string[] {
    const chunks: string[] = [];
    const delimiters = ['\\n\\n', '\\. ', '! ', '? ', '; ', ': '];

    // Try different delimiters until we get reasonable chunks
    for (const delimiter of delimiters) {
      const parts = content.split(new RegExp(delimiter, 'g'));

      if (parts.length > 1) {
        let currentChunk = '';

        for (const part of parts) {
          const testChunk = currentChunk ? `${currentChunk} ${part}` : part;

          if (this.getTextLength(testChunk) <= this.config.chunkSize) {
            currentChunk = testChunk;
          } else {
            // If current chunk is empty, it means the part itself is too large
            if (currentChunk === '') {
              // Split the large part by character count
              const subChunks = this.splitLargeChunk(part, this.config.chunkSize);
              if (subChunks.length > 0) {
                if (chunks.length > 0 && this.config.overlapSize > 0) {
                  // Add overlap from the last chunk
                  const lastChunk = chunks[chunks.length - 1];
                  const overlap = this.getOverlap(lastChunk.content, this.config.overlapSize);
                  subChunks[0] = overlap + subChunks[0];
                }
                chunks.push(...subChunks.map(text => text));
                currentChunk = subChunks[subChunks.length - 1];
              }
            } else {
              // Add the current chunk to results
              chunks.push(currentChunk);

              // Handle overlap
              if (chunks.length > 0 && this.config.overlapSize > 0) {
                const lastChunk = chunks[chunks.length - 1];
                const overlap = this.getOverlap(lastChunk, this.config.overlapSize);
                currentChunk = overlap + part;
              } else {
                currentChunk = part;
              }
            }
          }
        }

        // Add the last chunk if it exists
        if (currentChunk) {
          chunks.push(currentChunk);
        }

        // If we have chunks and they're reasonable sized, return them
        if (chunks.length > 0) {
          return chunks;
        }
      }
    }

    // Fallback: if no delimiters worked, split by character count
    return this.splitByCharacterCount(content, this.config.chunkSize, this.config.overlapSize);
  }

  private splitLargeChunk(text: string, maxSize: number): string[] {
    const chunks: string[] = [];
    for (let i = 0; i < text.length; i += maxSize) {
      chunks.push(text.substring(i, Math.min(i + maxSize, text.length)));
    }
    return chunks;
  }

  private splitByCharacterCount(content: string, chunkSize: number, overlapSize: number): string[] {
    const chunks: string[] = [];
    for (let i = 0; i < content.length; i += chunkSize - overlapSize) {
      const chunk = content.substring(i, Math.min(i + chunkSize, content.length));
      chunks.push(chunk);
    }
    return chunks;
  }

  private getTextLength(text: string): number {
    // Simple character count for now, could be enhanced with token counting
    return text.length;
  }

  private getOverlap(text: string, overlapSize: number): string {
    if (text.length <= overlapSize) {
      return text;
    }
    return text.slice(-overlapSize);
  }

  private createHeadingMap(headings: Array<{level: number, text: string, id?: string}>): Map<number, {level: number, text: string, id?: string}> {
    const headingMap = new Map<number, {level: number, text: string, id?: string}>();

    headings.forEach(heading => {
      // For now, we'll map approximate positions in the content
      // In a real implementation, we'd map based on actual positions
    });

    return headingMap;
  }

  private findRelevantHeading(chunkText: string, headingMap: Map<number, {level: number, text: string, id?: string}>, url: string): string | undefined {
    // Simple implementation: return the page title as the heading
    // In a real implementation, we would find the most relevant heading based on position
    return url.split('/').pop()?.replace(/-/g, ' ') || undefined;
  }
}