<?php

// proof that even PHP on the terminal can do basic softbody, too :)

const W = 80;
const H = 80;
const DT = 0.016;
const FLOOR_Y = 75.0;

/**
 * Simple 2D vector with immutable and in-place arithmetic.
 */
class Vec2 {
    public function __construct(public float $x, public float $y) {}

    public function add(Vec2 $o): Vec2     { return new Vec2($this->x + $o->x, $this->y + $o->y); }
    public function sub(Vec2 $o): Vec2     { return new Vec2($this->x - $o->x, $this->y - $o->y); }
    public function scale(float $s): Vec2  { return new Vec2($this->x * $s, $this->y * $s); }
    public function length(): float        { return sqrt($this->x * $this->x + $this->y * $this->y); }

    public function addEq(Vec2 $o): void   { $this->x += $o->x; $this->y += $o->y; }
    public function subEq(Vec2 $o): void   { $this->x -= $o->x; $this->y -= $o->y; }
    public function scaleEq(float $s): void { $this->x *= $s; $this->y *= $s; }
}

/**
 * 2D softbody made of point masses connected by Hooke springs.
 * Uses simple Euler integration with boundary collision.
 */
class SoftBody {
    /**
     * Position of each point mass.
     */
    public array $pos = [];

    /**
     * Velocity of each point mass.
     */
    public array $vel = [];

    /**
     * Index pairs [a, b] for each spring.
     */
    public array $springs = [];

    /**
     * Rest length for each spring, auto-computed from initial positions.
     */
    public array $restLengths = [];

    public float $gravity = 50.0;
    public float $damping = 0.98;
    public float $stiffness = 300.0;
    public float $bounce = 0.5;
    public float $friction = 0.9;

    public function __construct(array $positions, array $springs) {
        foreach ($positions as [$x, $y]) {
            $this->pos[] = new Vec2($x, $y);
            $this->vel[] = new Vec2(0, 0);
        }
        foreach ($springs as [$a, $b]) {
            $this->springs[] = [$a, $b];
            $this->restLengths[] = $this->pos[$a]->sub($this->pos[$b])->length();
        }
    }

    /**
     * Apply an instantaneous velocity impulse to a point mass.
     */
    public function applyImpulse(int $idx, Vec2 $impulse): void {
        $this->vel[$idx]->addEq($impulse);
    }

    /**
     * Step the simulation: gravity, spring forces, Euler integration, boundary collision.
     */
    public function update(float $dt): void {
        $n = count($this->pos);

        // gravity (F=ma, but mass explicitly 1 so we can simplify)
        $grav = new Vec2(0, $this->gravity * $dt);
        for ($i = 0; $i < $n; $i++) {
            $this->vel[$i]->addEq($grav);
        }

        // springs (Hooke's law: F = -k * (|x| - rest) * dir)
        foreach ($this->springs as $s => [$a, $b]) {
            $delta = $this->pos[$b]->sub($this->pos[$a]);
            $dist = $delta->length();
            if ($dist < 0.001) continue;
            $force = $delta->scale($this->stiffness * ($dist - $this->restLengths[$s]) / $dist * $dt);
            $this->vel[$a]->addEq($force);
            $this->vel[$b]->subEq($force);
        }

        // integrate + boundary collision
        for ($i = 0; $i < $n; $i++) {
            $this->vel[$i]->scaleEq($this->damping);
            $this->pos[$i]->addEq($this->vel[$i]->scale($dt));

            if ($this->pos[$i]->y > FLOOR_Y) {
                $this->pos[$i]->y = FLOOR_Y;
                $this->vel[$i]->y *= -$this->bounce;
                $this->vel[$i]->x *= $this->friction;
            }
            if ($this->pos[$i]->y < 1) {
                $this->pos[$i]->y = 1;
                $this->vel[$i]->y *= -$this->bounce;
            }
            if ($this->pos[$i]->x < 1) {
                $this->pos[$i]->x = 1;
                $this->vel[$i]->x *= -$this->bounce;
            }
            if ($this->pos[$i]->x > W-2) {
                $this->pos[$i]->x = W-2;
                $this->vel[$i]->x *= -$this->bounce;
            }
        }
    }

    /**
     * Rasterize the softbody into an ASCII buffer.
     * Springs are drawn as '.' and point masses as 'o'.
     */
    public function draw(array &$buf): void {
        foreach ($this->springs as [$a, $b]) {
            self::drawLine($buf, $this->pos[$a], $this->pos[$b]);
        }
        foreach ($this->pos as $p) {
            $px = (int)round($p->x);
            $py = (int)round($p->y);
            if ($px >= 0 && $px < W && $py >= 0 && $py < H) {
                $buf[$py][$px] = 'o';
            }
        }
    }

    /**
     * Bresenham-style line rasterization between two points.
     */
    private static function drawLine(array &$buf, Vec2 $from, Vec2 $to): void {
        $steps = (int)(max(abs($to->x - $from->x), abs($to->y - $from->y)) * 1.5);
        if ($steps < 1) $steps = 1;
        for ($i = 0; $i <= $steps; $i++) {
            $t = $i / $steps;
            $px = (int)round($from->x + ($to->x - $from->x) * $t);
            $py = (int)round($from->y + ($to->y - $from->y) * $t);
            if ($px >= 0 && $px < W && $py >= 0 && $py < H) {
                $buf[$py][$px] = '.';
            }
        }
    }
}

// --- setup ---

$body = new SoftBody(
    [[35,20], [45,20], [45,30], [35,30]],
    [[0,1], [1,2], [2,3], [3,0], [0,2], [1,3]]
);
$body->applyImpulse(0, new Vec2(30, 0));
$body->applyImpulse(1, new Vec2(0, 20));

// terminal raw mode
system('stty -icanon -echo');
stream_set_blocking(STDIN, false);
echo "\033[?25l";
register_shutdown_function(function() {
    echo "\033[?25h";
    system('stty sane');
});

// --- main loop ---

$frame = 0;
while (true) {
    // poll for keyboard input (non-blocking)
    $key = fread(STDIN, 16);
    if ($key === 'q') break;

    // space: off-center upward kick with random horizontal force
    if ($key === ' ') {
        $rx = mt_rand(-100, 100);
        $body->applyImpulse(3, new Vec2($rx, -150));
        $body->applyImpulse(2, new Vec2($rx * 0.3, -45));
    }

    // step physics
    $body->update(DT);

    // clear buffer and draw floor
    $buf = array_fill(0, H, array_fill(0, W, ' '));
    for ($x = 0; $x < W; $x++) $buf[(int)FLOOR_Y][$x] = '_';

    // rasterize softbody into buffer
    $body->draw($buf);

    // flush frame to terminal (cursor home, overwrite in place)
    echo "\033[H";
    for ($y = 0; $y < H; $y++) {
        echo implode('', $buf[$y]) . "\033[K\n";
    }
    echo "Frame $frame | SPACE=kick  Q=quit\033[K";

    // ~60fps
    usleep(16000);
    $frame++;
}
