// Exercise the actual React panel with the real loopback bridge + synthetic HM01.
const http = require('http');
const fs = require('fs');
const path = require('path');
const {spawn} = require('child_process');
const webpack = require('webpack');
async function run() {
  const playwright = require(process.env.VISTA_PLAYWRIGHT || 'playwright');
  const build = path.resolve('unittest/.browser-build');
  await new Promise((resolve, reject) => webpack({mode: 'development', entry: path.resolve('unittest/browser_entry.tsx'),
    output: {path: build, filename: 'test.js'}, resolve: {extensions: ['.ts', '.tsx', '.js']},
    module: {rules: [{test: /\.tsx?$/, exclude: /node_modules/, use: {loader: 'ts-loader', options: {transpileOnly: true, compilerOptions: {noEmit: false}}}}]}
  }, (err, stats) => err || stats.hasErrors() ? reject(err || new Error(stats.toString())) : resolve()));
  const app = http.createServer((req, res) => {
    if (req.url === '/test.js') {res.setHeader('Content-Type', 'text/javascript; charset=utf-8'); return res.end(fs.readFileSync(path.join(build, 'test.js')));}
    res.setHeader('Content-Type', 'text/html; charset=utf-8'); res.end('<html><head><meta charset="utf-8"></head><body style="margin:0;background:#080f1a;color:white"><div id="root"></div><script src="/test.js"></script></body></html>');
  });
  await new Promise(resolve => app.listen(18102, '127.0.0.1', resolve));
  const demoPath = process.env.VISTA_DEMO_SCRIPT;
  if (!demoPath) { throw new Error('Set VISTA_DEMO_SCRIPT to DigitalTwinRepo/scripts/grafana_demo.py'); }
  let child;
  const start = async () => {
    child = spawn(process.env.VISTA_PYTHON || 'python', [demoPath, '--port', '18101', '--grafana-origin', 'http://127.0.0.1:18102']);
    await new Promise((resolve, reject) => {
      child.once('error', reject); child.once('exit', code => reject(new Error('Demo exited ' + code)));
      child.stdout.on('data', data => {if (String(data).includes('SYNTHETIC DEMO')) {resolve();}});
      child.stderr.on('data', data => process.stderr.write(data));
    });
  };
  let browser;
  try {
    await start();
    browser = await playwright.chromium.launch({headless: true, channel: process.env.VISTA_BROWSER_CHANNEL || 'msedge', args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
    const page = await browser.newPage({viewport: {width: 1100, height: 680}});
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.goto('http://127.0.0.1:18102');
    try { await page.getByRole('status').filter({hasText: 'DEMO — not live'}).waitFor({timeout: 10000}); }
    catch (e) {
      await page.screenshot({path: path.join(build, 'failure.png')});
      console.log('Panel status:', await page.locator('body').innerText(), 'Page errors:', errors);
      throw e;
    }
    await page.locator('canvas').screenshot({path: path.join(build, 'mesh.png')});
    await page.screenshot({path: path.join(build, 'panel.png')});
    await page.getByRole('button', {name: 'Reset view'}).click();
    child.kill();
    await page.waitForFunction(() => /fetch|Failed|HTTP|abort/i.test(document.querySelector('[role=status]').textContent), {timeout: 6000});
    await page.screenshot({path: path.join(build, 'disconnected.png')});
    await start();
    await page.getByRole('status').filter({hasText: 'DEMO — not live'}).waitFor();
    if (errors.length) {throw new Error(errors.join('\n'));}
    console.log('PASS: real browser panel renders synthetic HM01, reset, disconnect, reconnect; no page errors');
    console.log('Screenshots: ' + build);
  } finally {child?.kill(); await browser?.close(); await new Promise(resolve => app.close(resolve));}
}
run().catch(error => {console.error(error); process.exitCode = 1;});
