---
name: run-slimevr
description: Build, launch, drive and screenshot the SlimeVR server and GUI. Use when asked to run, start, or screenshot the SlimeVR app, to click through the GUI, to verify a change in the real app rather than in tests, or to exercise AutoBone / body proportions without physical trackers.
---

# Running SlimeVR

Two processes. The **server** is Kotlin/Java (Gradle, `:server:desktop:run`) and
listens on `21110`. The **GUI** is a React app that in development is served by
plain Vite, so it can be driven headlessly in a browser — there is no need to
build or launch Electron.

The agent path is `.claude/skills/run-slimevr/driver.mjs`, a Playwright REPL
that reads one command per line on stdin. It exists because the onboarding
flow cannot be driven with ordinary clicks — see Gotchas.

All paths below are relative to the repository root.

## Prerequisites

**JDK 17.** The build rejects newer JDKs, and the system default here is 26.
Every Gradle command needs the override:

```bash
export JAVA_HOME=/opt/homebrew/opt/openjdk@17
```

Submodules (`solarxr-protocol`, and flatbuffers inside it):

```bash
git submodule update --init --recursive
```

Driver dependencies — self-contained, and gitignored:

```bash
npm install --prefix .claude/skills/run-slimevr
```

The driver finds a cached Playwright Chromium on its own. If there is none:
`npx playwright install chromium`, or point at any Chrome with
`SLIMEVR_CHROMIUM=/path/to/chrome`.

## Build and launch

GUI dependencies. This is a **pnpm workspace** (`pnpm-workspace.yaml` covers
`gui` and `solarxr-protocol`), so packages live in the *repo-root*
`node_modules/.pnpm` even though you install from `gui/`:

```bash
cd gui && pnpm install --frozen-lockfile
```

Start both, in the background, from the repository root:

```bash
JAVA_HOME=/opt/homebrew/opt/openjdk@17 ./gradlew :server:desktop:run --console=plain > /tmp/slimevr-server.log 2>&1 &
(cd gui && pnpm start > /tmp/slimevr-gui.log 2>&1 &)
```

The server takes ~45 s on a cold Gradle build. Wait for both:

```bash
grep -m1 "Bridge started on port" /tmp/slimevr-server.log
sed -E $'s/\e\\[[0-9;]*m//g' /tmp/slimevr-gui.log | grep -oE 'http://localhost:[0-9]+' | head -1
```

**Read the GUI port from the log rather than assuming 5173** — Vite silently
takes the next free port when something else holds it.

**Strip the ANSI escapes first.** Vite colours the port separately from the
rest of the URL, so the log literally contains
`http://localhost:<ESC>[1m5173`, and a plain
`grep -oE 'http://localhost:[0-9]+'` matches nothing at all — it looks like
Vite never started.

## Run (agent path)

Pipe commands into the driver. It prints a line per command, so the output is
the transcript of what happened:

```bash
printf '%s\n' \
  'goto http://localhost:5173/#/onboarding/body-proportions/auto' \
  'buttons' \
  'shot /tmp/shots/step1.png' \
  'click I have all my trackers on' \
  'buttons' \
  'quit' | node .claude/skills/run-slimevr/driver.mjs
```

Real output from that run:

```
[driver] chromium: /Users/…/ms-playwright/chromium-1223/…/Google Chrome for Testing
[goto] http://localhost:5173/#/onboarding/body-proportions/auto
[buttons centred] ["ESC","I have all my trackers on","Previous step"]
[shot] /tmp/shots/step1.png
[click] "I have all my trackers on" ok
[buttons centred] ["ESC","Previous step","I have read the requirements","Previous step"]
```

Commands: `goto <url>`, `shot <path>`, `click <exact button text>`, `buttons`
(only those fully inside the viewport), `allbuttons`, `text [n]`,
`find <regex>`, `waitfor <regex> [seconds]`, `wait <ms>`, `eval <js>`, `quit`.

Routing is a `HashRouter`, so routes are `#/…`:
`#/`, `#/settings/trackers`, `#/onboarding/body-proportions/auto`,
`#/onboarding/trackers-assign`.

**Always look at the screenshot.** A Vite module-resolution failure renders as
a white "Internal Server Error" page that still screenshots successfully.

## Run (human path)

`cd gui && pnpm gui` runs the Electron shell. Useless headlessly and not needed
for verification — the same React app is what Vite serves.

## Driving AutoBone without trackers

AutoBone's *record* path is gated on `PoseRecorder.isReadyToRecord`, which is
`server.trackersCount > 0`, so with no hardware it fails immediately with
"The server is not ready to record". Its *process* path replays a saved
recording instead and exercises the identical solver and RPC.

Put a `.pfr` or `.pfs` in the load directory — note this is resolved from the
OS config dir, **not** the server's working directory:

```
~/Library/Application Support/dev.slimevr.SlimeVR/Load AutoBone Recordings/   # macOS
$XDG_CONFIG_HOME/dev.slimevr.SlimeVR/Load AutoBone Recordings/                # Linux
```

Server settings not exposed in the GUI go in `server/desktop/vrconfig.yml`
(the working directory of `:server:desktop:run`), and are read at startup:

```yaml
autoBone:
  useLevenbergMarquardt: true
  lmMaxFramePairs: 200
```

Triggering *process* from the GUI needs a button that sends it; the only one
is reached after a successful recording. For a local run, temporarily point
`StartRecording.tsx`'s `start()` at `startProcessing()` instead of
`startRecording()` — Vite hot-reloads it — and revert afterwards.

To generate a recording with known bone lengths, drive a `HumanPoseManager`
with chosen offsets and translate the headset each frame so the ankles land on
the same point, then write it with `PfrIO.writeToFile`. `AutoBoneLeastSquaresTests`
does exactly this.

## Gotchas

- **Ordinary clicks do not work on the onboarding flow.** It is a
  transform-based carousel: every step is mounted at all times and the
  off-screen panels still take part in hit testing, so Playwright reports
  `<div class="… transition-transform"> intercepts pointer events` and times
  out on a button that is plainly visible. The driver's `click` dispatches
  `el.click()` inside the page, which bypasses hit testing and still fires
  React's handler.
- **`nextStep` is "+1 from wherever the slider is", not "go to my step".**
  Because every step is mounted, clicking a *later* step's advance button from
  step 0 moves the slider to step 1, not to that step. Advance one step at a
  time and confirm with `buttons` after each.
- **`buttons` is the only reliable read on carousel position.** `text` returns
  every step's content at once, including panels nobody can see, so it will
  happily show you results that are not on screen.
- **The Preparation step cannot be advanced without trackers.** Its advance
  control is a Full Reset button, which is disabled. Use another step's advance
  button to move past it — `nextStep` being relative makes this work.
- **Never delete the repo-root `node_modules`.** It is the pnpm workspace store;
  removing it breaks the already-running Vite with `Cannot find module …/vite/dist/node/chunks/…`.
  Recover with `cd gui && pnpm install --frozen-lockfile` and restart Vite.
- **WebGL fails headlessly.** "Rendering disabled / Failed to initialize WebGL"
  is expected; only the 3D avatar is missing and the rest of the UI is fine.
- **The server logs `ReadAfterEOFException` immediately at startup.** It is the
  interactive console reader hitting a closed stdin when backgrounded. Harmless.
- **Playwright's pinned browser is usually not the cached one.** Launching
  without `executablePath` fails with "Executable doesn't exist at
  …chromium_headless_shell-<n>". The driver picks the newest cached build.
- **Config lands in two places.** `vrconfig.yml` is read from the process
  working directory (`server/desktop/`), while AutoBone recording directories
  resolve to the OS config dir. They do not agree, so check both.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Unsupported class file major version` / toolchain error | `export JAVA_HOME=/opt/homebrew/opt/openjdk@17` |
| `Could not find :solarxr-protocol` or missing flatbuffers | `git submodule update --init --recursive` |
| `ERR_PNPM_NO_SCRIPT_OR_SERVER Missing script start` | You ran `pnpm start` at the repo root; it lives in `gui/` |
| `ERR_MODULE_NOT_FOUND: playwright` | `npm install --prefix .claude/skills/run-slimevr` — ESM ignores `NODE_PATH` |
| Browser "Internal Server Error", `Cannot find module …/vite/dist/…` | Root `node_modules` was deleted; reinstall and restart Vite |
| Vite looks like it never started — port grep returns nothing | ANSI escapes split the URL; strip them first (see Build and launch) |
| `locator.click: Timeout … intercepts pointer events` | Use the driver's `click`, not Playwright's |
| `[click] "…" NOT FOUND` | Label must match the button's visible text exactly; check with `allbuttons` |
| AutoBone: "The server is not ready to record" | No trackers connected — use the process path above |
| AutoBone: "No recordings found in …" | The `.pfr` must be in the OS config dir, not `server/desktop/` |

## Stopping

```bash
pkill -f "dev.slimevr.desktop.Main"; pkill -f vite
```
