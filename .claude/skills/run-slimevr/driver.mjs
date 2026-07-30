#!/usr/bin/env node
/**
 * Browser driver for the SlimeVR GUI.
 *
 * Reads one command per line on stdin and prints the result, so it works
 * piped (deterministic, scriptable) or interactively. Commands:
 *
 *   goto <url>            navigate and settle
 *   shot <path>           screenshot to a file
 *   click <label>         click a button by its exact visible text
 *   buttons               list buttons fully inside the viewport
 *   allbuttons            list every button, on-screen or not
 *   text [n]              first n characters of body innerText (default 1200)
 *   find <regex>          print body text around the first match
 *   waitfor <regex> [sec] poll body text until it matches (default 60s)
 *   wait <ms>             sleep
 *   eval <js>             run JS in the page, print the JSON result
 *   quit
 *
 * ## Why `click` goes through the DOM
 *
 * The onboarding flow is a transform-based carousel: every step is mounted
 * at all times and the off-screen ones still take part in hit testing, so
 * Playwright's normal click reports "<div …> intercepts pointer events" and
 * times out even on a button it can see. Dispatching `el.click()` inside the
 * page bypasses hit testing and still triggers React's handler.
 *
 * Use `buttons` (viewport-clipped) to know which panel is actually centred --
 * that is the only reliable read on where the carousel is.
 */
import { chromium } from 'playwright';
import { createInterface } from 'node:readline';
import { existsSync, readdirSync } from 'node:fs';
import { homedir } from 'node:os';
import { join } from 'node:path';

/**
 * Playwright refuses to launch when the npm package's pinned browser build is
 * not the one in the cache, which is the normal state on a machine where the
 * browsers were installed by some other project. Any cached Chromium drives
 * this app fine, so take the newest one rather than downloading another.
 */
function findChromium() {
  if (process.env.SLIMEVR_CHROMIUM) return process.env.SLIMEVR_CHROMIUM;
  const cache =
    process.platform === 'darwin'
      ? join(homedir(), 'Library', 'Caches', 'ms-playwright')
      : join(homedir(), '.cache', 'ms-playwright');
  if (!existsSync(cache)) return undefined;
  const candidates = [];
  for (const dir of readdirSync(cache)) {
    if (!dir.startsWith('chromium-')) continue;
    for (const rel of [
      'chrome-mac-arm64/Google Chrome for Testing.app/Contents/MacOS/Google Chrome for Testing',
      'chrome-mac/Chromium.app/Contents/MacOS/Chromium',
      'chrome-linux/chrome',
    ]) {
      const p = join(cache, dir, rel);
      if (existsSync(p)) candidates.push([parseInt(dir.split('-')[1], 10) || 0, p]);
    }
  }
  candidates.sort((a, b) => b[0] - a[0]);
  return candidates[0]?.[1];
}

const exe = findChromium();
if (!exe) {
  console.error(
    'no cached Chromium found. Set SLIMEVR_CHROMIUM=/path/to/chrome, or run: npx playwright install chromium'
  );
  process.exit(1);
}
console.log(`[driver] chromium: ${exe}`);

const browser = await chromium.launch({ executablePath: exe });
const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
page.on('pageerror', (e) => console.log('[pageerror]', String(e).slice(0, 200)));

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const listButtons = (clipToViewport) =>
  page.evaluate((clip) => {
    const out = [];
    for (const b of document.querySelectorAll('button')) {
      const r = b.getBoundingClientRect();
      const inView =
        r.width > 0 &&
        r.left >= 0 &&
        r.right <= window.innerWidth &&
        r.top >= 0 &&
        r.bottom <= window.innerHeight;
      if (clip && !inView) continue;
      const t = b.innerText.replace(/\s+/g, ' ').trim();
      if (t) out.push(t);
    }
    return out;
  }, clipToViewport);

const bodyText = () => page.evaluate(() => document.body.innerText);

async function run(line) {
  const [cmd, ...rest] = line.trim().split(/\s+/);
  const arg = line.trim().slice(cmd.length).trim();
  switch (cmd) {
    case '':
    case '#':
      return;
    case 'goto':
      await page.goto(arg, { waitUntil: 'domcontentloaded' });
      await sleep(4000);
      console.log(`[goto] ${page.url()}`);
      return;
    case 'shot':
      await page.screenshot({ path: arg });
      console.log(`[shot] ${arg}`);
      return;
    case 'click': {
      const ok = await page.evaluate((label) => {
        const b = [...document.querySelectorAll('button')].find(
          (el) => el.innerText.replace(/\s+/g, ' ').trim() === label
        );
        if (!b) return false;
        b.click();
        return true;
      }, arg);
      console.log(`[click] "${arg}" ${ok ? 'ok' : 'NOT FOUND'}`);
      await sleep(1400);
      return;
    }
    case 'buttons':
      console.log('[buttons centred] ' + JSON.stringify(await listButtons(true)));
      return;
    case 'allbuttons':
      console.log('[buttons all] ' + JSON.stringify(await listButtons(false)));
      return;
    case 'text': {
      const n = parseInt(rest[0] || '1200', 10);
      console.log((await bodyText()).slice(0, n));
      return;
    }
    case 'find': {
      const t = await bodyText();
      const m = t.match(new RegExp(arg));
      if (!m) return console.log(`[find] no match for /${arg}/`);
      const i = t.indexOf(m[0]);
      console.log(t.slice(Math.max(0, i - 100), i + 800));
      return;
    }
    case 'waitfor': {
      const parts = arg.split(/\s+/);
      const seconds = parseInt(parts[parts.length - 1], 10);
      const hasTimeout = Number.isFinite(seconds);
      const re = new RegExp(hasTimeout ? parts.slice(0, -1).join(' ') : arg);
      const limit = hasTimeout ? seconds : 60;
      for (let i = 0; i < limit / 2; i++) {
        await sleep(2000);
        if (re.test(await bodyText())) return console.log(`[waitfor] matched after ~${(i + 1) * 2}s`);
      }
      console.log(`[waitfor] TIMEOUT after ${limit}s`);
      return;
    }
    case 'wait':
      await sleep(parseInt(arg, 10));
      return;
    case 'eval':
      console.log('[eval] ' + JSON.stringify(await page.evaluate(arg)));
      return;
    case 'quit':
      return 'quit';
    default:
      console.log(`[driver] unknown command: ${cmd}`);
  }
}

const rl = createInterface({ input: process.stdin, terminal: false });
for await (const line of rl) {
  try {
    if ((await run(line)) === 'quit') break;
  } catch (e) {
    console.log('[error] ' + String(e).split('\n')[0]);
  }
}
await browser.close();
