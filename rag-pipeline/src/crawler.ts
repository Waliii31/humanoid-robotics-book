// src/crawler.ts
import axios from 'axios';
import * as cheerio from 'cheerio';
import { CrawledPage, CrawlConfig } from './types';
import { RetryHandler, NetworkErrorHandler } from './utils';

export class WebCrawler {
  private visitedUrls = new Set<string>();
  private pendingUrls: string[] = [];
  private crawledPages: CrawledPage[] = [];

  constructor(private config: CrawlConfig) {}

  async crawl(): Promise<CrawledPage[]> {
    // Initialize with the starting URLs
    this.pendingUrls = [...this.config.urls];

    while (this.pendingUrls.length > 0) {
      const url = this.pendingUrls.shift();
      if (!url || this.visitedUrls.has(url)) {
        continue;
      }

      this.visitedUrls.add(url);

      try {
        console.log(`Crawling: ${url}`);

        const page = await this.fetchAndParsePage(url);
        this.crawledPages.push(page);

        // Extract additional links if within max depth
        if (this.config.maxDepth !== undefined && this.getDepth(url) < this.config.maxDepth) {
          const links = this.extractLinks(page.content, url);
          this.pendingUrls.push(...links.filter(link => !this.visitedUrls.has(link)));
        }

        // Respect rate limiting
        if (this.config.rateLimitMs) {
          await this.delay(this.config.rateLimitMs);
        }
      } catch (error) {
        console.error(`Failed to crawl ${url}:`, error);
        // Continue with other URLs even if one fails
      }
    }

    return this.crawledPages;
  }

  private async fetchAndParsePage(url: string): Promise<CrawledPage> {
    // Using Puppeteer for JavaScript-heavy sites or Axios for simpler content
    return await RetryHandler.executeWithRetry(async () => {
      const response = await axios.get(url, {
        headers: {
          'User-Agent': 'Mozilla/5.0 (compatible; Docusaurus-RAG-Crawler/1.0)'
        },
        timeout: 15000 // Increased timeout
      });

      const $ = cheerio.load(response.data);

      // Extract content using configured selectors
      const contentSelector = this.config.selectors?.content || 'article, .markdown, .docItemContainer, main';
      const contentElement = $(contentSelector);

      // Remove excluded elements
      if (this.config.selectors?.excludeSelectors) {
        this.config.selectors.excludeSelectors.forEach(selector => {
          contentElement.find(selector).remove();
        });
      }

      const content = contentElement.text()
        .replace(/\s+/g, ' ')
        .trim();

      const title = $('title').text() || url.split('/').pop() || 'Untitled';

      // Extract headings
      const headings: Array<{level: number, text: string, id?: string}> = [];
      $('h1, h2, h3, h4, h5, h6').each((_, element) => {
        const $el = $(element);
        const level = parseInt($el[0].tagName.charAt(1));
        const text = $el.text().trim();
        const id = $el.attr('id') || undefined;

        if (text) {
          headings.push({ level, text, id });
        }
      });

      return {
        url,
        title,
        content,
        headings,
        metadata: {
          timestamp: new Date(),
          source: 'docusaurus'
        },
        createdAt: new Date()
      };
    }, 3, 2000); // 3 retries with 2s base delay
  }

  private extractLinks(content: string, baseUrl: string): string[] {
    const $ = cheerio.load(content);
    const links: string[] = [];

    // Find all anchor tags
    $('a[href]').each((_, element) => {
      const href = $(element).attr('href');
      if (href) {
        try {
          // Convert relative URLs to absolute
          const absoluteUrl = new URL(href, baseUrl).href;

          // Only include links from the same domain/base URL
          const baseDomain = new URL(baseUrl).hostname;
          const linkDomain = new URL(absoluteUrl).hostname;

          if (linkDomain === baseDomain && absoluteUrl.startsWith(baseUrl.replace(/\/$/, ''))) {
            links.push(absoluteUrl);
          }
        } catch (e) {
          // Skip invalid URLs
        }
      }
    });

    return links;
  }

  private getDepth(url: string): number {
    const baseUrl = new URL(this.config.urls[0]);
    const pathParts = new URL(url).pathname.split('/').filter(Boolean);
    return pathParts.length;
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}