import { defineConfig } from 'vite';
import fs from 'node:fs';
import path from 'node:path';

const SHOT_DIR = path.resolve(process.cwd(), 'qa/shots');

/**
 * Dev-only middleware so the automated visual-QA pass can write full-resolution
 * screenshots straight to disk (POST /qa/shot with { name, dataUrl }).
 * Same-origin, so no CORS dance.
 */
function qaCapture() {
  return {
    name: 'qa-capture',
    configureServer(server) {
      fs.mkdirSync(SHOT_DIR, { recursive: true });
      server.middlewares.use('/qa/shot', (req, res) => {
        if (req.method !== 'POST') { res.statusCode = 405; return res.end('POST only'); }
        let body = '';
        req.on('data', (c) => { body += c; });
        req.on('end', () => {
          try {
            const { name, dataUrl } = JSON.parse(body);
            const safe = String(name).replace(/[^a-z0-9._-]/gi, '_');
            const b64 = dataUrl.slice(dataUrl.indexOf(',') + 1);
            const file = path.join(SHOT_DIR, safe.endsWith('.png') ? safe : safe + '.png');
            fs.writeFileSync(file, Buffer.from(b64, 'base64'));
            res.setHeader('content-type', 'application/json');
            res.end(JSON.stringify({ ok: true, file, bytes: b64.length }));
          } catch (err) {
            res.statusCode = 500;
            res.end(JSON.stringify({ ok: false, error: String(err) }));
          }
        });
      });
    },
  };
}

export default defineConfig({
  plugins: [qaCapture()],
  // Yandex Games serves the uploaded build from a per-draft path, so every asset
  // reference has to be relative to index.html rather than to the origin root.
  base: './',
  server: { port: Number(process.env.PORT) || 5178, host: '127.0.0.1' },
  build: { target: 'esnext', sourcemap: false },
});
