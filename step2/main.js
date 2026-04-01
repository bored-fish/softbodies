// ============================================================================
// main.js - A Single Particle With Gravity
// ============================================================================
//
// Building on step 1, we now use proper physics:
//   - Gravity pulls the particle downward each frame.
//   - Euler integration turns forces into motion.
//   - The particle bounces off walls with energy loss.
//
// This uses integration.js which provides:
//   - Particle(x, y)         — creates a particle with position and velocity.
//   - applyGravity(body, g)  — adds gravitational force to each particle.
//   - integrate(body, dt)    — updates velocity and position from forces.
//   - enforceWorldBounds()   — bounces particles off the world edges.
// ============================================================================

// ---- Configuration ---------------------------------------------------------

const WORLD_WIDTH = 10;         // Meters.
const WORLD_HEIGHT = 7.5;       // Meters.

const GRAVITY = -9.8;           // m/s^2 downward.
const BOUNCE = 0.3;             // Coefficient of restitution for wall bounces.

const MAX_DT = 1 / 30;          // Clamp dt to avoid explosions on tab-away.

// ---- World Boundaries ------------------------------------------------------

const bounds = {
    left: 0,
    right: WORLD_WIDTH,
    bottom: 0,
    top: WORLD_HEIGHT
};

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

// ---- Create the particle ---------------------------------------------------

// We wrap the single particle in a "body" object with a particles array,
// because that's the shape integration.js expects.
function createBody() {
    return { particles: [new Particle(5, 5)] };
}

let body = createBody();

document.getElementById('reset').addEventListener('click', () => {
    body = createBody();
    dragParticle = null;
});

// ---- Rendering -------------------------------------------------------------

function draw() {
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // World boundary.
    ctx.strokeStyle = '#444466';
    ctx.lineWidth = 1;
    ctx.strokeRect(
        toScreenX(bounds.left),
        toScreenY(bounds.top),
        WORLD_WIDTH * scale,
        WORLD_HEIGHT * scale
    );

    // The particle.
    const p = body.particles[0];
    ctx.fillStyle = '#ff6b6b';
    ctx.beginPath();
    ctx.arc(toScreenX(p.x), toScreenY(p.y), 6, 0, Math.PI * 2);
    ctx.fill();
}

// ---- Input (mouse drag) ----------------------------------------------------

let dragParticle = null;
const GRAB_RADIUS = 0.5;

function toWorldX(sx) { return (sx - offsetX) / scale; }
function toWorldY(sy) { return WORLD_HEIGHT - (sy - offsetY) / scale; }

canvas.addEventListener('mousedown', (e) => {
    const wx = toWorldX(e.clientX);
    const wy = toWorldY(e.clientY);
    const p = body.particles[0];
    const dx = p.x - wx;
    const dy = p.y - wy;
    if (dx * dx + dy * dy < GRAB_RADIUS * GRAB_RADIUS) {
        dragParticle = p;
    }
});

canvas.addEventListener('mousemove', (e) => {
    if (!dragParticle) return;
    dragParticle.x = toWorldX(e.clientX);
    dragParticle.y = toWorldY(e.clientY);
    dragParticle.vx = 0;
    dragParticle.vy = 0;
});

canvas.addEventListener('mouseup', () => {
    dragParticle = null;
});

// ---- Simulation Loop -------------------------------------------------------

let lastTime = 0;

function tick(now) {
    requestAnimationFrame(tick);

    const dt = Math.min((now - lastTime) / 1000, MAX_DT);
    lastTime = now;

    // --- Step 1: Apply gravity ---
    applyGravity(body, GRAVITY);

    // --- Step 2: Integrate (Euler) ---
    integrate(body, dt);

    // --- Step 3: World boundaries ---
    enforceWorldBounds(body, bounds, BOUNCE);

    // --- Step 4: Render ---
    draw();
}

requestAnimationFrame(tick);
