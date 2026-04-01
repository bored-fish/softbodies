// ============================================================================
// main.js - Setup, Rendering, and Input
// ============================================================================
//
// This file ties the three physics modules together:
//   - integration.js: Euler integration, gravity, world bounds.
//   - joints.js:      Spring constraints between particles.
//   - collision.js:   Softbody-vs-softbody collision.
//
// The simulation loop each frame:
//   1. Apply gravity to all bodies.
//   2. Solve spring joints (edge + cross bracing).
//   3. Integrate positions with Euler.
//   4. Enforce world boundaries.
//   5. Detect and resolve body-vs-body collisions.
//   6. Render.
// ============================================================================

// ---- Configuration ---------------------------------------------------------

const WORLD_WIDTH = 10;         // Meters.
const WORLD_HEIGHT = 7.5;       // Meters.

const GRAVITY = -9.8;           // m/s^2 downward.
const SPRING_STIFFNESS = 500;   // N/m (spring constant for joints).
const SPRING_DAMPING = 5;       // Ns/m (damping in the springs).

const ELASTICITY = 0.3;         // Coefficient of restitution (0 = clay, 1 = rubber).
const FRICTION = 0.2;

const MAX_DT = 1 / 30;          // Clamp dt to avoid explosions on tab-away.

// ---- World Boundaries (in simulation coordinates) --------------------------

const bounds = {
    left: 0,
    right: WORLD_WIDTH,
    bottom: 0,
    top: WORLD_HEIGHT
};

// ---- Body Colors -----------------------------------------------------------

const BODY_COLORS = [
    '#ff6b6b',
    '#4ecdc4',
    '#ffe66d',
    '#a29bfe',
    '#55efc4',
    '#fd79a8',
    '#74b9ff',
    '#ffeaa7',
    '#dfe6e9',
];

// ---- Create a Softbody Cube ------------------------------------------------

// Create a rectangular softbody at the given position.
// Four particles (corners) connected by six joints:
//   - 4 edge joints (the sides).
//   - 2 cross joints (the diagonals, to prevent shearing).
//
// Particles are ordered counter-clockwise:
//   0: bottom-left, 1: bottom-right, 2: top-right, 3: top-left
function createRect(cx, cy, w, h) {
    const hw = w / 2;
    const hh = h / 2;

    // Four corners, counter-clockwise from bottom-left.
    const particles = [
        new Particle(cx - hw, cy - hh), // 0: bottom-left
        new Particle(cx + hw, cy - hh), // 1: bottom-right
        new Particle(cx + hw, cy + hh), // 2: top-right
        new Particle(cx - hw, cy + hh), // 3: top-left
    ];

    const diag = Math.sqrt(w * w + h * h);

    const joints = [
        // Edge joints (the 4 sides).
        new Joint(0, 1, w),    // bottom
        new Joint(1, 2, h),    // right
        new Joint(2, 3, w),    // top
        new Joint(3, 0, h),    // left

        // Cross joints (the 2 diagonals) - these prevent shearing.
        new Joint(0, 2, diag),
        new Joint(1, 3, diag),
    ];

    return { particles, joints };
}

// ---- Canvas Setup ----------------------------------------------------------

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// Scale factor: how many pixels per meter.
let scale = 1;
let offsetX = 0;
let offsetY = 0;

function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;

    // Fit the world into the window, preserving aspect ratio.
    const scaleX = canvas.width / WORLD_WIDTH;
    const scaleY = canvas.height / WORLD_HEIGHT;
    scale = Math.min(scaleX, scaleY);

    // Center the world in the window.
    offsetX = (canvas.width - WORLD_WIDTH * scale) / 2;
    offsetY = (canvas.height - WORLD_HEIGHT * scale) / 2;
}

resize();
window.addEventListener('resize', resize);

// Convert simulation coordinates (meters, Y-up) to canvas pixels (Y-down).
function toScreenX(x) { return offsetX + x * scale; }
function toScreenY(y) { return offsetY + (WORLD_HEIGHT - y) * scale; }

// ---- Create Bodies ---------------------------------------------------------

function createBodies() {
    return [
        createRect(1.2, 3.5, 0.7, 0.7),    // 1: square
        createRect(2.5, 3.5, 0.9, 0.55),   // 2: wide
        createRect(3.8, 3.5, 0.5, 0.85),   // 3: tall
        createRect(5.0, 3.5, 0.65, 0.65),  // 4: square
        createRect(6.2, 3.5, 1.0, 0.45),   // 5: wide flat
        createRect(7.5, 3.5, 0.6, 0.8),    // 6: tall-ish
        createRect(8.7, 3.5, 0.55, 0.55),  // 7: small square
        createRect(4.5, 5.5, 0.8, 0.5),    // 8: wide, starts higher
    ];
}

let bodies = createBodies();

document.getElementById('reset').addEventListener('click', () => {
    bodies = createBodies();
    dragParticle = null;
});

// ---- Rendering -------------------------------------------------------------

function draw() {
    // Clear.
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

    // Draw each body.
    for (let bi = 0; bi < bodies.length; bi++) {
        const body = bodies[bi];
        const color = BODY_COLORS[bi % BODY_COLORS.length];
        const n = body.particles.length;

        // Filled polygon.
        ctx.fillStyle = color + '40'; // 25% opacity via hex alpha.
        ctx.beginPath();
        ctx.moveTo(toScreenX(body.particles[0].x), toScreenY(body.particles[0].y));
        for (let i = 1; i < n; i++) {
            ctx.lineTo(toScreenX(body.particles[i].x), toScreenY(body.particles[i].y));
        }
        ctx.closePath();
        ctx.fill();

        // Joint lines (faint).
        ctx.strokeStyle = color + '4d'; // ~30% opacity.
        ctx.lineWidth = 1;
        for (const j of body.joints) {
            const a = body.particles[j.a];
            const b = body.particles[j.b];
            ctx.beginPath();
            ctx.moveTo(toScreenX(a.x), toScreenY(a.y));
            ctx.lineTo(toScreenX(b.x), toScreenY(b.y));
            ctx.stroke();
        }

        // Outline.
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(toScreenX(body.particles[0].x), toScreenY(body.particles[0].y));
        for (let i = 1; i < n; i++) {
            ctx.lineTo(toScreenX(body.particles[i].x), toScreenY(body.particles[i].y));
        }
        ctx.closePath();
        ctx.stroke();

        // Particle dots.
        ctx.fillStyle = color;
        for (let i = 0; i < n; i++) {
            const sx = toScreenX(body.particles[i].x);
            const sy = toScreenY(body.particles[i].y);
            ctx.beginPath();
            ctx.arc(sx, sy, 3, 0, Math.PI * 2);
            ctx.fill();
        }
    }
}

// ---- Input (mouse drag) ----------------------------------------------------

let dragParticle = null; // The particle currently being dragged, or null.
const GRAB_RADIUS = 0.3; // Meters — how close the click needs to be to grab.

// Convert screen pixel coordinates back to simulation meters.
function toWorldX(sx) { return (sx - offsetX) / scale; }
function toWorldY(sy) { return WORLD_HEIGHT - (sy - offsetY) / scale; }

canvas.addEventListener('mousedown', (e) => {
    const wx = toWorldX(e.clientX);
    const wy = toWorldY(e.clientY);

    // Find the closest particle to the click.
    let bestDist = GRAB_RADIUS * GRAB_RADIUS;
    for (const body of bodies) {
        for (const p of body.particles) {
            const dx = p.x - wx;
            const dy = p.y - wy;
            const distSq = dx * dx + dy * dy;
            if (distSq < bestDist) {
                bestDist = distSq;
                dragParticle = p;
            }
        }
    }
});

canvas.addEventListener('mousemove', (e) => {
    if (!dragParticle) return;

    // Move the particle to the mouse position and zero its velocity
    // so it doesn't fly off when released.
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

    // Compute real elapsed time from the timestamp provided by requestAnimationFrame.
    // On first frame lastTime is 0, so clamp to avoid a huge initial step.
    const dt = Math.min((now - lastTime) / 1000, MAX_DT);
    lastTime = now;

    // --- Step 1: Apply gravity ---
    for (const body of bodies) {
        applyGravity(body, GRAVITY);
    }

    // --- Step 2: Solve spring joints ---
    for (const body of bodies) {
        solveJoints(body, SPRING_STIFFNESS, SPRING_DAMPING);
    }

    // --- Step 3: Integrate (Euler) ---
    for (const body of bodies) {
        integrate(body, dt);
    }

    // --- Step 4: World boundaries ---
    for (const body of bodies) {
        enforceWorldBounds(body, bounds, ELASTICITY);
    }

    // --- Step 5: Body-vs-body collisions ---
    solveCollisions(bodies, ELASTICITY, FRICTION);

    // --- Step 6: Render ---
    draw();
}

requestAnimationFrame(tick);
