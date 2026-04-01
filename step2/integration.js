// ============================================================================
// integration.js - Euler Integration for Point Masses
// ============================================================================
//
// Each softbody is a collection of point masses (particles). Every frame we:
//   1. Apply gravity to each particle.
//   2. Integrate velocity and position using forward Euler.
//   3. Bounce particles off the world boundaries.
//
// Forward Euler is the simplest integrator:
//   velocity += acceleration * dt
//   position += velocity * dt
//
// It's not the most stable (Verlet or RK4 are better), but it's dead simple
// to understand and good enough for what we're doing here.
// ============================================================================

// A single point mass in the simulation.
// Mass is always 1 kg — kept implicit so we don't need to deal with it.
function Particle(x, y) {
    this.x = x;
    this.y = y;
    this.vx = 0;
    this.vy = 0;

    // Forces are accumulated each frame, then zeroed after integration.
    this.fx = 0;
    this.fy = 0;
}

// Apply gravity to every particle in a body.
// Since mass is 1, F = g directly.
function applyGravity(body, gravity) {
    for (const p of body.particles) {
        p.fy += gravity;
    }
}

// Integrate all particles using Euler.
// velocity += force * dt   (since mass is 1, force IS acceleration)
// position += velocity * dt
// Forces are zeroed after so they can be freshly accumulated next frame.
function integrate(body, dt) {
    for (const p of body.particles) {
        // With mass = 1, acceleration equals force directly.
        const ax = p.fx;
        const ay = p.fy;

        // Update velocity: v += a * dt
        p.vx += ax * dt;
        p.vy += ay * dt;

        // Light damping to bleed off energy and keep things stable.
        // Without this, Euler integration tends to gain energy over time.
        p.vx *= 0.999;
        p.vy *= 0.999;

        // Update position: x += v * dt
        p.x += p.vx * dt;
        p.y += p.vy * dt;

        // Zero accumulated forces for the next frame.
        p.fx = 0;
        p.fy = 0;
    }
}

// Bounce particles off the world boundaries.
// When a particle passes a boundary, clamp it back and reflect its
// velocity with some energy loss.
function enforceWorldBounds(body, bounds, bounce) {
    for (const p of body.particles) {
        if (p.x < bounds.left) {
            p.x = bounds.left;
            p.vx = Math.abs(p.vx) * bounce;
        } else if (p.x > bounds.right) {
            p.x = bounds.right;
            p.vx = -Math.abs(p.vx) * bounce;
        }

        if (p.y < bounds.bottom) {
            p.y = bounds.bottom;
            p.vy = Math.abs(p.vy) * bounce;
        } else if (p.y > bounds.top) {
            p.y = bounds.top;
            p.vy = -Math.abs(p.vy) * bounce;
        }
    }
}
