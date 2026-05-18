#include "esp_web.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "esp_config.h"
#include "esp_telemetry.h"

static WebServer server(80);

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NXP Robot Control</title>
  <style>
    :root {
      --red: #AA0000;
      --red-bright: #d11a1a;
      --red-soft: #6f0000;
      --black: #050505;
      --panel: #101010;
      --white: #f3f3f3;
      --muted: #c7c7c7;
      --line: rgba(170, 0, 0, 0.35);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background:
        radial-gradient(circle at top, rgba(170,0,0,0.18), transparent 28%),
        linear-gradient(160deg, #0c0c0c 0%, #040404 48%, #100000 100%);
      color: var(--white);
    }
    .shell {
      max-width: 1400px;
      margin: 0 auto;
      padding: 20px;
    }
    .hero {
      display: flex;
      justify-content: space-between;
      align-items: end;
      gap: 16px;
      padding: 18px 20px;
      border: 1px solid rgba(170,0,0,0.35);
      background: linear-gradient(180deg, rgba(170,0,0,0.16), rgba(10,10,10,0.94));
      border-radius: 20px;
      box-shadow: 0 18px 60px rgba(0,0,0,0.4);
      margin-bottom: 18px;
    }
    h1 {
      margin: 0;
      font-size: clamp(28px, 4vw, 48px);
      letter-spacing: 0.04em;
    }
    .sub {
      color: var(--muted);
      margin-top: 8px;
    }
    .status {
      padding: 8px 12px;
      border-radius: 999px;
      background: rgba(0,0,0,0.55);
      border: 1px solid rgba(170,0,0,0.55);
      color: var(--white);
      min-width: 240px;
      text-align: right;
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 12px;
    }
    .esp-badge { display:flex; align-items:center; gap:10px; }
    .esp-circle {
      width:44px; height:44px; border-radius:50%;
      display:flex; align-items:center; justify-content:center;
      background: linear-gradient(180deg, rgba(255,255,255,0.06), rgba(255,255,255,0.02));
      border: 1px solid rgba(255,255,255,0.02);
      font-weight:700; color:var(--white);
      font-size:14px;
      box-shadow: 0 6px 18px rgba(0,0,0,0.5);
    }
    .esp-led { width:14px; height:14px; border-radius:50%; box-shadow: 0 2px 8px rgba(0,0,0,0.6); border:1px solid rgba(0,0,0,0.6); }
    .esp-led.red { background: #9b0000; box-shadow: 0 0 8px rgba(155,0,0,0.75); }
    .esp-led.green { background: #1abc57; box-shadow: 0 0 8px rgba(26,188,87,0.8); }
    .status-text { color:var(--muted); font-size:13px; text-align:right; min-width:110px; }
    .grid {
      display: grid;
      grid-template-columns: 340px 1fr;
      gap: 18px;
    }
    .card {
      background: rgba(8,8,8,0.92);
      border: 1px solid rgba(170,0,0,0.28);
      border-radius: 20px;
      padding: 18px;
      box-shadow: 0 16px 45px rgba(0,0,0,0.35);
    }
    .card h2 {
      margin: 0 0 14px;
      font-size: 20px;
      color: #fff;
    }
    .buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 10px;
    }
    .btn {
      border: 0;
      border-radius: 14px;
      padding: 16px 12px;
      font-size: 16px;
      font-weight: 700;
      color: var(--white);
      background: linear-gradient(180deg, var(--red-bright), var(--red));
      box-shadow: 0 10px 24px rgba(170,0,0,0.3);
      cursor: pointer;
      transition: transform 0.12s ease, filter 0.12s ease;
      user-select: none;
    }
    .btn:hover { filter: brightness(1.06); transform: translateY(-1px); }
    .btn:active { transform: translateY(1px) scale(0.99); }
    .btn.secondary {
      background: linear-gradient(180deg, #2d2d2d, #151515);
      border: 1px solid rgba(170,0,0,0.5);
      box-shadow: none;
    }
    .btn.stop {
      background: linear-gradient(180deg, #f0f0f0, #bdbdbd);
      color: #080808;
    }
    .controls-layout {
      display: grid;
      gap: 10px;
      grid-template-columns: repeat(3, 1fr);
      margin-top: 10px;
    }
    .controls-layout .up { grid-column: 2; }
    .controls-layout .left { grid-column: 1; }
    .controls-layout .stop { grid-column: 2; }
    .controls-layout .right { grid-column: 3; }
    .controls-layout .down { grid-column: 2; }
    .metrics {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
      margin-top: 14px;
    }
    .metric {
      padding: 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.03);
      border: 1px solid rgba(170,0,0,0.18);
    }
    .metric .label {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .metric .value {
      margin-top: 6px;
      font-size: 18px;
      color: #fff;
      word-break: break-word;
    }
    .canvas-wrap {
      background: #000;
      border: 1px solid rgba(170,0,0,0.35);
      border-radius: 20px;
      overflow: hidden;
      min-height: 640px;
      position: relative;
    }
    .canvas-head {
      padding: 14px 18px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid rgba(170,0,0,0.25);
      background: linear-gradient(180deg, rgba(170,0,0,0.12), rgba(0,0,0,0.92));
    }
    .canvas-head .hint {
      color: var(--muted);
      font-size: 13px;
    }
    canvas {
      display: block;
      width: 100%;
      height: 560px;
      background: #000;
    }
    @media (max-width: 960px) {
      .grid { grid-template-columns: 1fr; }
      .status { text-align: left; min-width: unset; width: 100%; }
      .hero { flex-direction: column; align-items: stretch; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <div>
        <h1>NXP Robot Control</h1>
        <div class="sub">Real-time UART telemetry, on-screen manual drive controls, and a live trajectory visualization derived from Teensy telemetry data.</div>
      </div>
      <div class="status" id="status">
        <div class="esp-badge">
          <div class="esp-circle">ESP</div>
          <div id="espLed" class="esp-led red" aria-hidden="true"></div>
        </div>
        <div id="statusText" class="status-text">Connecting…</div>
      </div>
    </section>

    <div class="grid">
      <section class="card">
        <h2>Manual Control</h2>
        <div class="controls-layout">
          <button class="btn up" onclick="sendCommand('F')">Forward</button>
          <button class="btn left" onclick="sendCommand('L')">Left</button>
          <button class="btn stop" onclick="sendCommand('S')">Stop</button>
          <button class="btn right" onclick="sendCommand('R')">Right</button>
          <button class="btn down" onclick="sendCommand('B')">Backward</button>
        </div>

        <div class="metrics">
          <div class="metric"><div class="label">Vx</div><div class="value" id="vx">0.0000</div></div>
          <div class="metric"><div class="label">Vy</div><div class="value" id="vy">0.0000</div></div>
          <div class="metric"><div class="label">Steering</div><div class="value" id="steering">0.00</div></div>
          <div class="metric"><div class="label">Servo</div><div class="value" id="servo">0.00</div></div>
        </div>
      </section>

      <section class="canvas-wrap">
        <div class="canvas-head">
          <div>
            <strong>Real-Time Ride Trace</strong>
          </div>
          <div class="hint">Live path</div>
        </div>
        <canvas id="trailCanvas"></canvas>
      </section>
    </div>
  </div>

  <script>
    const statusEl = document.getElementById('status');
    const statusTextEl = document.getElementById('statusText');
    const espLedEl = document.getElementById('espLed');
    const vxEl = document.getElementById('vx');
    const vyEl = document.getElementById('vy');
    const steeringEl = document.getElementById('steering');
    const servoEl = document.getElementById('servo');
    const canvas = document.getElementById('trailCanvas');
    const ctx = canvas.getContext('2d');
    let lastPoints = [];

    function resizeCanvas() {
      const ratio = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * ratio));
      canvas.height = Math.max(1, Math.floor(rect.height * ratio));
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      renderTrail(lastPoints);
    }

    function drawGrid(width, height) {
      ctx.save();
      ctx.strokeStyle = 'rgba(170, 0, 0, 0.18)';
      ctx.lineWidth = 1;
      for (let x = 0; x < width; x += 40) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }
      for (let y = 0; y < height; y += 40) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }
      ctx.restore();
    }

    function renderTrail(points) {
      const rect = canvas.getBoundingClientRect();
      const width = rect.width;
      const height = rect.height;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#000000';
      ctx.fillRect(0, 0, width, height);
      drawGrid(width, height);

      if (!points || points.length < 2) {
        ctx.save();
        ctx.fillStyle = 'rgba(255,255,255,0.7)';
        ctx.font = '16px Arial';
        ctx.fillText('Waiting for telemetry...', 18, 28);
        ctx.restore();
        return;
      }

      let minX = points[0].x, maxX = points[0].x;
      let minY = points[0].y, maxY = points[0].y;
      for (const p of points) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }

      const pad = 28;
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
      const offsetX = (width - spanX * scale) / 2 - minX * scale;
      const offsetY = (height - spanY * scale) / 2 - minY * scale;

      ctx.save();
      ctx.strokeStyle = '#f5f5f5';
      ctx.lineWidth = 3;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.shadowColor = 'rgba(255,255,255,0.4)';
      ctx.shadowBlur = 8;
      ctx.beginPath();
      points.forEach((p, index) => {
        const x = p.x * scale + offsetX;
        const y = p.y * scale + offsetY;
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      const first = points[0];
      const last = points[points.length - 1];
      const firstX = first.x * scale + offsetX;
      const firstY = first.y * scale + offsetY;
      const lastX = last.x * scale + offsetX;
      const lastY = last.y * scale + offsetY;

      ctx.shadowBlur = 0;
      ctx.fillStyle = '#AA0000';
      ctx.beginPath();
      ctx.arc(firstX, firstY, 6, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(lastX, lastY, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    async function sendCommand(command) {
      try {
        await fetch(`/cmd?c=${encodeURIComponent(command)}`);
        statusTextEl.textContent = `Sent command: ${command}`;
      } catch (error) {
        statusTextEl.textContent = 'Command failed';
      }
    }

    function bindHoldButton(selector, command) {
      const button = document.querySelector(selector);
      let engaged = false;

      const start = (event) => {
        event.preventDefault();
        engaged = true;
        sendCommand(command);
      };

      const stop = (event) => {
        if (!engaged) {
          return;
        }
        event.preventDefault();
        engaged = false;
        sendCommand('S');
      };

      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', stop);
      button.addEventListener('pointerleave', stop);
      button.addEventListener('touchcancel', stop);
    }

    async function refreshData() {
      try {
        const response = await fetch('/data');
        const data = await response.json();
        const latest = data.latest || {};
        lastPoints = data.points || [];
        vxEl.textContent = Number(latest.vx || 0).toFixed(4);
        vyEl.textContent = Number(latest.vy || 0).toFixed(4);
        steeringEl.textContent = Number(latest.steering || 0).toFixed(2);
        servoEl.textContent = Number(latest.servo || 0).toFixed(2);
        statusTextEl.textContent = `Live packets: ${lastPoints.length}`;
        if (espLedEl) { espLedEl.classList.remove('red'); espLedEl.classList.add('green'); }
        renderTrail(lastPoints);
      } catch (error) {
        if (espLedEl) { espLedEl.classList.remove('green'); espLedEl.classList.add('red'); }
        statusTextEl.textContent = 'No telemetry';
      }
    }

    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
    refreshData();
    setInterval(refreshData, 100);
    bindHoldButton('.up', 'F');
    bindHoldButton('.down', 'B');
    bindHoldButton('.left', 'L');
    bindHoldButton('.right', 'R');
    document.querySelector('.stop').addEventListener('click', () => sendCommand('S'));
    document.addEventListener('keydown', (event) => {
      const keyMap = { ArrowUp: 'F', ArrowDown: 'B', ArrowLeft: 'L', ArrowRight: 'R', Space: 'S' };
      if (keyMap[event.code]) {
        event.preventDefault();
        sendCommand(keyMap[event.code]);
      }
    });
    document.addEventListener('keyup', (event) => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.code)) {
        event.preventDefault();
        sendCommand('S');
      }
    });
  </script>
</body>
</html>
)rawliteral";

static void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleData()
{
  server.send(200, "application/json", EspTelemetry_BuildJson());
}

static void handleCommand()
{
  String command = server.arg("c");
  if (command.length() == 0)
  {
    server.send(400, "text/plain", "missing command");
    return;
  }

  char driveCommand = command.charAt(0);
  EspTelemetry_SendCommand(driveCommand);
  server.send(200, "text/plain", "ok");
}

void EspWeb_Init()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ESP_AP_SSID, ESP_AP_PASSWORD);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCommand);
  server.begin();
}

void EspWeb_Loop()
{
  server.handleClient();
}