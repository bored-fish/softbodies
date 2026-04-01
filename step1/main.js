// ============================================================================
// main.js - The Simplest Possible Moving Particle
// ============================================================================
//
// No physics engine, no forces, no integration. Just:
//
//   position = position + speed
//
// When the particle hits a wall, we flip its direction. That's it.
// This is the "hello world" of particle simulation.
// ============================================================================

// ---- Configuration ---------------------------------------------------------

const WORLD_WIDTH = 10;         // Meters.
const WORLD_HEIGHT = 7.5;       // Meters.
const SPEED = 3;                // Meters per second.

// ---- The Particle ----------------------------------------------------------

// Position: starts in the center.
let px = 5;
let py = 3.75;

// Direction: normalized, angled so it doesn't just go perfectly horizontal.
let dx = 0.7;
let dy = 0.5;

function resetParticle() {
    px = 5;
    py = 3.75;
    dx = 0.7;
    dy = 0.5;
}

// ---- Canvas Setup ----------------------------------------------------------

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let scale = 1;
let offsetX = 0;
let offsetY = 0;

function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;

    const scaleX = canvas.width / WORLD_WIDTH;
    const scaleY = canvas.height / WORLD_HEIGHT;
    scale = Math.min(scaleX, scaleY);

    offsetX = (canvas.width - WORLD_WIDTH * scale) / 2;
    offsetY = (canvas.height - WORLD_HEIGHT * scale) / 2;
}

resize();
window.addEventListener('resize', resize);

function toScreenX(x) { return offsetX + x * scale; }
function toScreenY(y) { return offsetY + (WORLD_HEIGHT - y) * scale; }

document.getElementById('reset').addEventListener('click', resetParticle);

// ---- Simulation Loop -------------------------------------------------------

let lastTime = 0;

function tick(now) {
    requestAnimationFrame(tick);

    // Time since last frame, in seconds. Clamped to avoid huge jumps.
    const dt = Math.min((now - lastTime) / 1000, 1 / 30);
    lastTime = now;

    // ---- Update ----

    // Move the particle: position = position + direction * speed * dt
    px += dx * SPEED * dt;
    py += dy * SPEED * dt;

    // Bounce off walls: if we've passed a boundary, flip that axis.
    if (px < 0)            { px = 0;            dx = -dx; }
    if (px > WORLD_WIDTH)  { px = WORLD_WIDTH;  dx = -dx; }
    if (py < 0)            { py = 0;            dy = -dy; }
    if (py > WORLD_HEIGHT) { py = WORLD_HEIGHT; dy = -dy; }

    // ---- Draw ----

    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // World boundary.
    ctx.strokeStyle = '#444466';
    ctx.lineWidth = 1;
    ctx.strokeRect(
        toScreenX(0),
        toScreenY(WORLD_HEIGHT),
        WORLD_WIDTH * scale,
        WORLD_HEIGHT * scale
    );

    // The particle.
    ctx.fillStyle = '#ff6b6b';
    ctx.beginPath();
    ctx.arc(toScreenX(px), toScreenY(py), 6, 0, Math.PI * 2);
    ctx.fill();
}

requestAnimationFrame(tick);
