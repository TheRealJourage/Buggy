// Prints the cylinder box drawing set (viewer/zeichnungen.html) to viewer/downloads/Zylinderbox-Zeichnungen.pdf.
// Needs the viewer on :8112 and Playwright with Chromium:
//   PLAYWRIGHT=/path/to/node_modules/playwright node tools/zylinderbox_pdf.mjs [vault-copy.pdf]
import { createRequire } from 'node:module';
import { copyFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT || 'playwright');
const url = process.env.URL || 'http://127.0.0.1:8112/zeichnungen.html';
const out = fileURLToPath(new URL('../viewer/downloads/Zylinderbox-Zeichnungen.pdf', import.meta.url));

const browser = await chromium.launch();
const page = await browser.newPage();
await page.goto(url, { waitUntil: 'networkidle' });
await page.waitForFunction(() => window.drawingsReady === true);
await page.pdf({ path: out, width: '420mm', height: '297mm', printBackground: true, margin: { top: 0, right: 0, bottom: 0, left: 0 } });
await browser.close();
console.log('written', out);
if (process.argv[2]) { copyFileSync(out, process.argv[2]); console.log('copied to', process.argv[2]); }
