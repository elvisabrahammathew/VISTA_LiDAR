const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const http = require('node:http');
const webpack = require('webpack');
async function run() {
  const {chromium} = require(process.env.VISTA_PLAYWRIGHT || 'playwright');
  const build = path.resolve('unittest/.resize-build');
  await new Promise((resolve, reject) => webpack({mode: 'development', entry: path.resolve('unittest/resize_entry.tsx'),
    output: {path: build, filename: 'test.js'}, resolve: {extensions: ['.ts', '.tsx', '.js']},
    module: {rules: [{test: /\.tsx?$/, exclude: /node_modules/, use: {loader: 'ts-loader', options: {transpileOnly: true, compilerOptions: {noEmit: false}}}}]}
  }, (error, stats) => error || stats.hasErrors() ? reject(error || new Error(stats.toString())) : resolve()));
  const server = http.createServer((req, res) => {
    if (req.url === '/test.js') {
      res.setHeader('Content-Type', 'text/javascript; charset=utf-8');
      return res.end(fs.readFileSync(path.join(build, 'test.js')));
    }
    res.setHeader('Content-Type', 'text/html; charset=utf-8');
    res.end('<html><head><meta charset="utf-8"></head><body style="margin:12px;background:#111217;color:#ddd;font:14px Arial"><div id="root"></div><script src="/test.js"></script></body></html>');
  });
  await new Promise(resolve => server.listen(18103, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({channel: 'msedge', headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
    for (const ratio of [.75, 1, 1.5, 2]) {
      const context = await browser.newContext({deviceScaleFactor: ratio, viewport: {width: 1480, height: 650}});
      const page = await context.newPage(), errors = [];
      page.on('pageerror', error => errors.push(error.message));
      await page.goto('http://127.0.0.1:18103');
      await page.locator('canvas').nth(1).waitFor();
      for (const [width, height] of [[600, 440], [700, 520], [460, 300]]) {
        await page.evaluate(({width, height}) => window.resizePanels(width, height), {width, height});
        await page.waitForFunction(({width, height}) => Array.from(document.querySelectorAll('canvas')).every(canvas => {
          const box = canvas.getBoundingClientRect();
          return Math.abs(box.width - width) < .1 && Math.abs(box.height - height) < .1;
        }), {width, height}, {timeout: 5000});
        const sizes = await page.locator('canvas').evaluateAll(canvases => canvases.map(canvas => ({
          width: canvas.width, height: canvas.height, css: canvas.getBoundingClientRect().toJSON(),
          parent: canvas.parentElement.getBoundingClientRect().toJSON()
        })));
        for (const size of sizes) {
          assert.equal(size.css.width, width); assert.equal(size.css.height, height);
          assert.equal(size.parent.width, width); assert.equal(size.parent.height, height);
          assert.equal(size.width, Math.floor(width * ratio)); assert.equal(size.height, Math.floor(height * ratio));
        }
        console.log(`PASS: both canvases fill ${width}x${height} at DPR ${ratio}`);
        if (ratio === .75 && width === 700) {await page.screenshot({path: path.join(build, 'full-panel-dpr075.png')});}
      }
      assert.deepEqual(errors, []); await context.close();
    }
    console.log('PASS: full panel canvas sizing at four pixel ratios, including dynamic resizing; no page errors');
  } finally {await browser?.close(); await new Promise(resolve => server.close(resolve));}
}
run().catch(error => {console.error(error); process.exitCode = 1;});
