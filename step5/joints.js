// ============================================================================
// joints.js - Spring Joints (Hooke's Law)
// ============================================================================
//
// Joints are springs that connect pairs of particles. They try to maintain
// a rest length by applying forces that push or pull the particles.
//
// We use Hooke's law with damping:
//   F_spring  = stiffness * (distance - restLength)
//   F_damping = damping * (relative velocity along the spring axis)
//   F_total   = (F_spring + F_damping) * direction
//
// Each body has two kinds of joints:
//   - Edge joints: connect adjacent corners (the 4 sides of the cube).
//   - Cross joints: connect opposite corners (the 2 diagonals).
//
// The cross joints prevent the cube from collapsing into a flat line.
// Without them, a square made of only edge springs can shear freely.
// ============================================================================

// A spring joint between two particles.
function Joint(a, b, restLength) {
    this.a = a;
    this.b = b;
    this.restLength = restLength;
}

// Apply spring forces for all joints in a body using Hooke's law.
// For each joint:
//   1. Find the vector between the two particles.
//   2. Compute displacement (currentLength - restLength).
//   3. Compute damping from the relative velocity along the spring.
//   4. Apply equal and opposite forces to both particles.
function solveJoints(body, stiffness, damping) {
    for (const joint of body.joints) {
        const a = body.particles[joint.a];
        const b = body.particles[joint.b];

        // Vector from a to b.
        const dx = b.x - a.x;
        const dy = b.y - a.y;

        // Current distance between the two particles.
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance < 0.0001) continue; // Avoid division by zero.

        // Unit direction vector from a toward b.
        const nx = dx / distance;
        const ny = dy / distance;

        // How far the spring is from its rest length.
        // Positive = stretched, negative = compressed.
        const displacement = distance - joint.restLength;

        // Relative velocity of b with respect to a, projected onto
        // the spring direction. This is the "closing speed" of the spring.
        const relativeSpeed = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny;

        // Hooke's law: F = k * x
        // Damping force: F = c * v (along spring axis)
        const springForce = stiffness * displacement;
        const dampingForce = damping * relativeSpeed;
        const totalForce = springForce + dampingForce;

        // Apply equal and opposite forces (Newton's third law).
        const fx = nx * totalForce;
        const fy = ny * totalForce;

        a.fx += fx;
        a.fy += fy;
        b.fx -= fx;
        b.fy -= fy;
    }
}
