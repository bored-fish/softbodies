// ============================================================================
// collision.js - Softbody vs Softbody Collision (Minimal Version)
// ============================================================================
//
// How do you detect when two wobbly shapes overlap, and push them apart?
//
// We do it in two stages:
//
//   1. DETECT  - For each particle of body A, check if it's inside body B.
//                If so, find the closest edge of B to push it out through.
//
//   2. RESOLVE - Push the particle out (position correction) and apply an
//                impulse to both bodies (velocity correction).
//
// A production system would add a "broad phase" (bounding box pre-check)
// so you don't waste time testing bodies that are nowhere near each other.
// We skip that here to keep things as simple as possible.
// ============================================================================


// ============================================================================
// POINT-IN-POLYGON
// ============================================================================

/**
 * Test whether a point is inside a polygon.
 *
 * Uses the "ray casting" algorithm:
 *   - Imagine shooting a horizontal laser beam from the point out to the right.
 *   - Count how many edges of the polygon the beam crosses.
 *   - If it crosses an ODD number of edges, the point is inside.
 *   - If EVEN (including zero), it's outside.
 *
 * Why does this work? Every time the ray enters the polygon it crosses an
 * edge, and every time it exits it crosses another. So inside = odd crossings.
 *
 *      Outside --|----> Inside --|----> Outside
 *             edge 1         edge 2
 *        (1 crossing = inside)  (2 crossings = outside again)
 */
function pointInPolygon(px, py, verts) {
    let inside = false;
    const n = verts.length;

    // Walk every edge of the polygon. Each edge connects vertex i to vertex j.
    // j starts at the last vertex so the first edge connects [last] -> [0].
    for (let i = 0, j = n - 1; i < n; j = i++) {
        const xi = verts[i].x, yi = verts[i].y;
        const xj = verts[j].x, yj = verts[j].y;

        // Two conditions for the ray to cross this edge:
        //   1. The edge must straddle the ray's y-coordinate. That is, one
        //      endpoint must be above py and the other below (or on) it.
        //   2. The crossing point must be to the RIGHT of px (since our ray
        //      shoots rightward).
        //
        // The second condition is checked by computing where the edge
        // crosses y=py and seeing if that x is greater than px.
        if ((yi > py) !== (yj > py) &&
            px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            // Each crossing toggles inside/outside.
            inside = !inside;
        }
    }

    return inside;
}


// ============================================================================
// CLOSEST POINT ON EDGE
// ============================================================================

/**
 * Find the closest point on a line segment to a given test point.
 *
 * Imagine the edge as a ruler lying in space. We want to find the spot on
 * that ruler that is nearest to our test point. Three cases:
 *
 *   - If the nearest spot is past the start of the ruler, clamp to the start.
 *   - If it's past the end, clamp to the end.
 *   - Otherwise, it's somewhere along the middle — we project onto the line.
 *
 * We also compute the edge's outward-facing NORMAL. This is a unit vector
 * pointing perpendicular to the edge, toward the outside of the body.
 * (Assumes counter-clockwise vertex winding.)
 *
 * Returns:
 *   hx, hy   - The closest point on the edge ("hit point").
 *   nx, ny   - The edge's outward normal.
 *   t        - How far along the edge the hit point is (0 = start, 1 = end).
 *   distSq   - Squared distance from the test point to the hit point.
 */
function closestPointOnEdge(px, py, ax, ay, bx, by) {
    // Vector along the edge, from start (a) to end (b).
    const dx = bx - ax;
    const dy = by - ay;
    const lenSq = dx * dx + dy * dy;

    // If the edge has zero length (two particles at the same spot), just
    // return the start point. This shouldn't happen in practice, but we
    // guard against division by zero.
    if (lenSq < 0.00001) {
        const ex = px - ax, ey = py - ay;
        return { hx: ax, hy: ay, nx: 0, ny: 0, t: 0, distSq: ex * ex + ey * ey };
    }

    // Length of the edge, and a unit vector along it.
    const len = Math.sqrt(lenSq);
    const dirX = dx / len;
    const dirY = dy / len;

    // The edge NORMAL — perpendicular to the edge direction.
    // For a counter-clockwise wound polygon, rotating the edge direction
    // 90 degrees clockwise gives an outward-facing normal.
    //
    //   Edge direction: (dirX, dirY)
    //   Rotated CW 90:  (dirY, -dirX)
    //
    // This is the direction we'll push the penetrating particle along.
    const nx = dirY;
    const ny = -dirX;

    // Vector from edge start to the test point.
    const toX = px - ax;
    const toY = py - ay;

    // Project that vector onto the edge direction. This tells us how far
    // along the edge the closest point is (in world units, not 0-1 yet).
    //
    //   projected = dot(toPoint, edgeDir)
    //
    // If projected < 0, the closest point is before the start of the edge.
    // If projected > len, it's past the end. Otherwise it's in the middle.
    const projected = toX * dirX + toY * dirY;

    // Clamp to the segment endpoints.
    let hx, hy, t;
    if (projected <= 0) {
        // Closest point is the start of the edge.
        hx = ax; hy = ay; t = 0;
    } else if (projected >= len) {
        // Closest point is the end of the edge.
        hx = bx; hy = by; t = 1;
    } else {
        // Closest point is somewhere along the middle of the edge.
        t = projected / len;        // Normalize to 0-1 range.
        hx = ax + dirX * projected; // Walk along the edge to that spot.
        hy = ay + dirY * projected;
    }

    // Squared distance from the test point to the hit point.
    // We use squared distance to avoid a sqrt — we only need to compare
    // distances, and comparing squared values gives the same ordering.
    const ex = px - hx, ey = py - hy;
    return { hx, hy, nx, ny, t, distSq: ex * ex + ey * ey };
}


// ============================================================================
// CONTACT COLLECTION
// ============================================================================
// For every particle of body A, we check if it's inside body B. If it is,
// we search all of B's edges to find the nearest one — that's the surface
// we'll push the particle out through. We record all the info we need into
// a "contact" object.
//
// Why do we need the edge info? Because when we push the particle out, we
// need to know:
//   - WHICH DIRECTION to push (the edge normal).
//   - HOW FAR to push (the penetration depth).
//   - WHERE on the edge the hit is (so we can distribute the response
//     across the edge's two endpoint particles).
// ============================================================================

/**
 * Collect contacts for one body's particles penetrating into another body.
 *
 * For each particle in bodyA, check if it's inside bodyB. If so, find the
 * closest edge of bodyB and record a contact.
 */
function collectContactsOneWay(bodyA, bodyB) {
    const contacts = [];
    const n = bodyB.particles.length;

    for (let pi = 0; pi < bodyA.particles.length; pi++) {
        const p = bodyA.particles[pi];

        // STEP 1: Is this particle inside bodyB?
        // If not, skip it — no collision for this particle.
        if (!pointInPolygon(p.x, p.y, bodyB.particles)) continue;

        // STEP 2: It IS inside. Now find the closest edge of bodyB.
        // That's the edge we'll push the particle back out through.
        // We check every edge and keep the one with the smallest distance.
        let bestDistSq = Infinity;
        let best = null;

        for (let ei = 0; ei < n; ei++) {
            // Each edge connects particle[ei] to particle[(ei+1) % n].
            // The modulo wraps around so the last particle connects back
            // to the first, closing the polygon.
            const ej = (ei + 1) % n;
            const a = bodyB.particles[ei];
            const b = bodyB.particles[ej];

            const hit = closestPointOnEdge(p.x, p.y, a.x, a.y, b.x, b.y);

            if (hit.distSq < bestDistSq) {
                bestDistSq = hit.distSq;
                best = {
                    bodyA,              // The body that owns the penetrating particle.
                    bodyB,              // The body that owns the edge being penetrated.
                    particleIndex: pi,  // Which particle of bodyA is inside.
                    edgeA: ei,          // First endpoint index of the closest edge.
                    edgeB: ej,          // Second endpoint index of the closest edge.
                    hx: hit.hx,         // The closest point on that edge (x).
                    hy: hit.hy,         // The closest point on that edge (y).
                    nx: hit.nx,         // Edge outward normal (x). Push direction.
                    ny: hit.ny,         // Edge outward normal (y). Push direction.
                    t: hit.t,           // 0-1 parameter along the edge (0 = edgeA, 1 = edgeB).
                    penetration: Math.sqrt(hit.distSq) // How deep the particle is inside.
                };
            }
        }

        if (best) contacts.push(best);
    }

    return contacts;
}


// ============================================================================
// RESOLUTION
// ============================================================================
// We now have a list of contacts. Each one says "this particle is stuck
// inside that body, and here's the nearest edge to push it out through."
//
// Resolution has two parts:
//
//   1. POSITION CORRECTION — physically separate the overlapping objects.
//      Nudge the penetrating particle outward along the edge normal, and
//      nudge the edge's particles inward (opposite direction). Since all
//      particles have equal mass, the penetrating particle gets pushed 1/3
//      of the way and the edge side gets pushed 2/3 (it has two particles).
//
//   2. VELOCITY CORRECTION — make them bounce/slide properly.
//      Without this, the objects would separate but then immediately fall
//      back into each other next frame. We compute an "impulse" (an
//      instantaneous change in velocity) that makes them bounce apart.
//
//      The impulse formula is:
//        J = -(1 + elasticity) * closingSpeed / (1/massA + 1/massB)
//
//      With unit masses this simplifies to:
//        J = -(1 + elasticity) * closingSpeed / 1.5
//
//      - closingSpeed: how fast they're moving toward each other along
//        the collision normal. Negative = closing, positive = separating.
//      - elasticity: 0 = dead stop (clay), 1 = perfect bounce (rubber).
//
//      We also apply a smaller FRICTION impulse along the surface tangent
//      to slow down sliding.
// ============================================================================

/**
 * Resolve all contacts by fixing positions and velocities.
 */
function resolveContacts(contacts, elasticity, friction) {
    for (const c of contacts) {
        // Grab the three particles involved in this contact:
        //   pA  = the penetrating particle (from bodyA).
        //   pB1 = first endpoint of the edge it's penetrating through (from bodyB).
        //   pB2 = second endpoint of that edge (from bodyB).
        const pA = c.bodyA.particles[c.particleIndex];
        const pB1 = c.bodyB.particles[c.edgeA];
        const pB2 = c.bodyB.particles[c.edgeB];

        // ---- Edge interpolation weights ----
        // The contact hit somewhere along the edge, parameterized by t (0 to 1).
        // t=0 means the hit is right at pB1, t=1 means right at pB2.
        // We use these weights to distribute forces across both endpoints
        // proportionally — if the hit is near pB1, pB1 gets most of the force.
        const b1w = 1 - c.t;   // Weight for pB1 (e.g. t=0.2 -> b1w=0.8).
        const b2w = c.t;       // Weight for pB2 (e.g. t=0.2 -> b2w=0.2).

        // ---- Surface velocity at the hit point ----
        // The edge is defined by two particles that might be moving differently.
        // To find the velocity of the surface AT the exact hit location, we
        // interpolate between the two endpoint velocities using the same weights.
        const bvx = pB1.vx * b1w + pB2.vx * b2w;
        const bvy = pB1.vy * b1w + pB2.vy * b2w;

        // ---- Relative velocity ----
        // How fast is the penetrating particle moving RELATIVE to the surface?
        // If both are moving right at 5 m/s, relative velocity is zero — no collision.
        // We only care about their motion relative to each other.
        const relVx = pA.vx - bvx;
        const relVy = pA.vy - bvy;

        // ---- Inverse masses ----
        // With all particles having mass 1:
        //   - The penetrating particle side has inverse mass 1/1 = 1.
        //   - The edge side has two particles, so combined mass 2, inverse 1/2.
        //   - Total inverse mass = 1 + 0.5 = 1.5.
        // These are constant, but written out so you can see where mass
        // would plug in if particles had different weights.
        const invMassA = 1;     // 1 / particleMass
        const invMassB = 0.5;   // 1 / (particleMass + particleMass)
        const invMassSum = 1.5; // invMassA + invMassB

        // ================================================================
        // PART 1: POSITION CORRECTION
        // ================================================================
        // The particle is inside the other body by `penetration` meters.
        // We need to push them apart by at least that much.
        //
        // We add a tiny bias (5mm) so they don't end up EXACTLY touching —
        // that would cause flickering as floating-point rounding makes them
        // alternate between "just inside" and "just outside" each frame.
        //
        // The push is split by inverse mass ratio:
        //   A gets 1/1.5 = 2/3 of the push (single particle, lighter side).
        //   B gets 0.5/1.5 = 1/3 of the push (two particles, heavier side).
        const separation = c.penetration + 0.005;
        const aPush = separation * (invMassA / invMassSum);
        const bPush = separation * (invMassB / invMassSum);

        // Push the penetrating particle OUT along the edge normal.
        pA.x += c.nx * aPush;
        pA.y += c.ny * aPush;

        // Push the edge particles IN (opposite direction), distributed
        // across the two endpoints by their interpolation weights.
        // If the hit was near pB1, pB1 moves more.
        pB1.x -= c.nx * bPush * b1w;
        pB1.y -= c.ny * bPush * b1w;
        pB2.x -= c.nx * bPush * b2w;
        pB2.y -= c.ny * bPush * b2w;

        // ================================================================
        // PART 2: VELOCITY CORRECTION
        // ================================================================
        // Fixing positions alone isn't enough. If the particle is still
        // moving INTO the surface, it'll just penetrate again next frame.
        // We need to change the velocities so they bounce apart.

        // ---- Are they actually closing? ----
        // Project the relative velocity onto the collision normal.
        // Negative = closing (moving into each other). Positive = separating.
        // If they're already separating, skip the impulse — they're fine.
        const relDotNormal = relVx * c.nx + relVy * c.ny;
        if (relDotNormal >= 0) continue; // Already moving apart, nothing to do.

        // ---- Normal impulse (bounce) ----
        // This is the core collision formula. It computes how much velocity
        // change is needed to make the objects separate.
        //
        //   J = -(1 + e) * v_closing / (1/mA + 1/mB)
        //   J = -(1 + e) * v_closing / 1.5
        //
        // - (1 + elasticity): elasticity=0 means "just stop closing" (the 1
        //   cancels the closing speed). elasticity=1 means "bounce back with
        //   the same speed" (1+1=2, so we reverse and match the closing speed).
        // - Dividing by 1.5 distributes the impulse: the single particle
        //   gets a bigger velocity change than each edge endpoint.
        const impulse = -(1 + elasticity) * relDotNormal / invMassSum;

        // ---- Tangential impulse (friction) ----
        // The tangent is perpendicular to the normal — it runs ALONG the
        // surface. Friction resists sliding along this direction.
        //
        // Think of it like this: the normal impulse handles the "bounce",
        // and the friction impulse handles the "grip".
        const tx = -c.ny; // Tangent direction: rotate normal 90 degrees.
        const ty = c.nx;
        const relDotTangent = relVx * tx + relVy * ty;
        const frictionImpulse = relDotTangent * friction / invMassSum;

        // ---- Apply impulses to the penetrating particle ----
        // Change its velocity by J * invMass in the normal direction (bounce)
        // and subtract the friction component in the tangent direction (grip).
        // With mass=1, invMassA=1, so the impulse applies directly.
        pA.vx += (c.nx * impulse - tx * frictionImpulse) * invMassA;
        pA.vy += (c.ny * impulse - ty * frictionImpulse) * invMassA;

        // ---- Apply opposite impulses to the edge endpoints ----
        // Newton's third law: every action has an equal and opposite reaction.
        // The edge gets pushed the OTHER way. The impulse is scaled by invMassB
        // (0.5, since the edge side has mass 2), then distributed across both
        // endpoints using the interpolation weights (b1w, b2w).
        pB1.vx -= (c.nx * impulse - tx * frictionImpulse) * invMassB * b1w;
        pB1.vy -= (c.ny * impulse - ty * frictionImpulse) * invMassB * b1w;
        pB2.vx -= (c.nx * impulse - tx * frictionImpulse) * invMassB * b2w;
        pB2.vy -= (c.ny * impulse - ty * frictionImpulse) * invMassB * b2w;
    }
}


// ============================================================================
// TOP-LEVEL COLLISION PIPELINE
// ============================================================================
// This is the function that main.js calls each frame. It checks every pair
// of bodies for collisions and resolves any it finds.
//
// No broad phase here — we just test every pair directly. This is O(n^2)
// and fine for a handful of bodies. A real game would add bounding box
// checks to skip pairs that are far apart.
// ============================================================================

/**
 * Run the collision pipeline between all pairs of bodies.
 */
function solveCollisions(bodies, elasticity, friction) {
    // Double loop that visits every unique pair exactly once.
    // i=0,j=1 then i=0,j=2 ... i=1,j=2 ... etc.
    // Starting j at i+1 avoids checking a body against itself and avoids
    // checking the same pair twice (A-B and B-A).
    for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
            const a = bodies[i];
            const b = bodies[j];

            // Collect contacts in both directions because either body's
            // particles could be poking into the other one.
            const contacts = [
                ...collectContactsOneWay(a, b),  // A's particles inside B.
                ...collectContactsOneWay(b, a)   // B's particles inside A.
            ];

            // Resolve all contacts for this pair.
            if (contacts.length > 0) {
                resolveContacts(contacts, elasticity, friction);
            }
        }
    }
}
