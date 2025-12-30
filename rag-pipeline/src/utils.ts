// src/utils.ts
export class RetryHandler {
  static async executeWithRetry<T>(
    operation: () => Promise<T>,
    maxRetries: number = 3,
    baseDelay: number = 1000,
    backoffMultiplier: number = 2
  ): Promise<T> {
    let lastError: Error;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        return await operation();
      } catch (error) {
        lastError = error as Error;

        if (attempt === maxRetries) {
          break;
        }

        // Calculate delay with exponential backoff
        const delay = baseDelay * Math.pow(backoffMultiplier, attempt);
        console.log(`Attempt ${attempt + 1} failed: ${lastError.message}. Retrying in ${delay}ms...`);

        await this.delay(delay);
      }
    }

    throw lastError!;
  }

  private static delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

export class NetworkErrorHandler {
  static isNetworkError(error: any): boolean {
    const errorMessage = error.message.toLowerCase();
    return (
      errorMessage.includes('network') ||
      errorMessage.includes('timeout') ||
      errorMessage.includes('fetch') ||
      errorMessage.includes('connect') ||
      error.code === 'ECONNREFUSED' ||
      error.code === 'ENOTFOUND' ||
      error.code === 'ECONNRESET' ||
      error.response?.status >= 500 // Server errors
    );
  }

  static isRateLimitError(error: any): boolean {
    return (
      error.response?.status === 429 ||
      error.message.toLowerCase().includes('rate limit') ||
      error.message.toLowerCase().includes('too many requests')
    );
  }
}