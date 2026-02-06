#!/usr/bin/env node

// Diagnostic script to check if RAG backend is properly configured
import 'dotenv/config';
import * as fs from 'fs';
import axios from 'axios';

interface CheckResult {
  name: string;
  status: 'pass' | 'warning' | 'fail';
  message: string;
}

const checks: CheckResult[] = [];

async function check(name: string, fn: () => Promise<boolean>, errorMsg: string) {
  try {
    const result = await fn();
    checks.push({
      name,
      status: result ? 'pass' : 'warning',
      message: result ? 'OK' : errorMsg,
    });
  } catch (error) {
    checks.push({
      name,
      status: 'fail',
      message: error instanceof Error ? error.message : String(error),
    });
  }
}

async function main() {
  console.log('\n🔍 RAG Backend Configuration Diagnostic\n');
  console.log('═'.repeat(50) + '\n');

  // Check .env file exists
  await check('.env file exists', async () => {
    const exists = fs.existsSync('.env');
    if (!exists) {
      console.log('  → Create it with: cp .env.example .env');
    }
    return exists;
  }, '.env file not found');

  // Check environment variables
  const envVars = [
    { name: 'QDRANT_URL', required: true },
    { name: 'QDRANT_API_KEY', required: true },
    { name: 'COHERE_API_KEY', required: true },
    { name: 'GEMINI_API_KEY', required: true },
  ];

  for (const envVar of envVars) {
    await check(
      `${envVar.name} is set`,
      async () => {
        const value = process.env[envVar.name];
        if (!value) {
          console.log(`  → Set in your .env file`);
          return false;
        }
        const masked = value.substring(0, 4) + '*'.repeat(Math.max(0, value.length - 8)) + value.substring(value.length - 4);
        console.log(`  → Value: ${masked}`);
        return true;
      },
      `${envVar.name} is not set`
    );
  }

  // Check Qdrant connectivity
  await check('Qdrant cluster is accessible', async () => {
    const qdrantUrl = process.env.QDRANT_URL;
    const qdrantApiKey = process.env.QDRANT_API_KEY;

    if (!qdrantUrl) return false;

    try {
      const response = await axios.get(`${qdrantUrl}/health`, {
        headers: {
          'api-key': qdrantApiKey || '',
        },
        timeout: 5000,
      });

      console.log(`  → Qdrant version: ${response.data?.version || 'unknown'}`);
      return response.status === 200;
    } catch (error) {
      throw new Error(`Cannot connect to Qdrant at ${qdrantUrl}`);
    }
  }, 'Cannot connect to Qdrant');

  // Check node_modules
  await check('Node dependencies are installed', async () => {
    return fs.existsSync('node_modules');
  }, 'Run: npm install');

  // Check TypeScript files
  await check('TypeScript source files exist', async () => {
    return (
      fs.existsSync('src/embedder.ts') &&
      fs.existsSync('src/chunker.ts') &&
      fs.existsSync('src/vector-store.ts')
    );
  }, 'Source files missing');

  // Check docs directory
  await check('Local docs directory exists', async () => {
    return fs.existsSync('../docs');
  }, 'Run this from backend/ directory');

  // Count markdown files
  await check('Documentation files are available', async () => {
    const docsPath = '../docs';
    let fileCount = 0;

    const walkDir = (dir: string) => {
      const files = fs.readdirSync(dir);
      for (const file of files) {
        const filePath = `${dir}/${file}`;
        const stat = fs.statSync(filePath);
        if (stat.isDirectory() && !file.startsWith('.')) {
          walkDir(filePath);
        } else if (file.endsWith('.md')) {
          fileCount++;
        }
      }
    };

    try {
      walkDir(docsPath);
      if (fileCount > 0) {
        console.log(`  → Found ${fileCount} markdown files`);
      }
      return fileCount > 0;
    } catch {
      return false;
    }
  }, 'No markdown files found');

  // Print results
  console.log('\n' + '═'.repeat(50));
  console.log('\n📊 Check Results:\n');

  let passed = 0;
  let warnings = 0;
  let failed = 0;

  for (const check of checks) {
    const icon = check.status === 'pass' ? '✅' : check.status === 'warning' ? '⚠️ ' : '❌';
    console.log(`${icon} ${check.name}`);
    if (check.message !== 'OK') {
      console.log(`   ${check.message}\n`);
    }

    if (check.status === 'pass') passed++;
    else if (check.status === 'warning') warnings++;
    else failed++;
  }

  console.log('\n' + '═'.repeat(50));
  console.log(`\n Summary: ${passed}/${checks.length} checks passed\n`);

  if (failed > 0) {
    console.log('❌ Issues found! Please fix before running ingestion.\n');
    process.exit(1);
  }

  if (warnings > 0) {
    console.log('⚠️  Warnings found. Setup may work but could have issues.\n');
  }

  if (failed === 0) {
    console.log('✅ All checks passed! Run the following commands:\n');
    console.log('  1. npm run ingest    # Populate Qdrant with documentation');
    console.log('  2. npm run dev       # Start the backend server\n');
    console.log('In another terminal, run:');
    console.log('  3. npm start         # Start Docusaurus frontend\n');
    console.log('Then visit http://localhost:3000 and use the chatbot!\n');
  }
}

main().catch(error => {
  console.error('\n❌ Diagnostic error:', error);
  process.exit(1);
});
