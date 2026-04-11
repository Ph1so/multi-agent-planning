#!/usr/bin/env python3
"""
Generate the "Gauntlet" map — a small, coordination-heavy parking lot.

── Why it's harder to coordinate ────────────────────────────────────────────
Agents enter from BOTH the top and bottom of the lot and are given goals on
the OPPOSITE side:

  • Top agents  (start y=18-21)  →  spots in the BOTTOM half (rows A & B)
  • Bottom agents (start y=2-5)  →  spots in the TOP half  (rows C & D)

Every agent must cross through the CENTRAL LANE (y=11-12), the only connection
between the two halves.  Top agents travel south while bottom agents travel
north — they meet head-on in that 2-cell-wide bottleneck, requiring explicit
conflict resolution that naive orderings cannot avoid.

── Layout (16 × 24 grid, y=0 at bottom) ─────────────────────────────────────

  y=23  ┌──────────────┐
  y=22  │  outer wall  │
  y=21  │ top queue    │  ← x=2-3 and x=12-13 only, ROAD
  y=20  │              │
  y=19  │              │
  y=18  │              │
  y=17  │ top access   │  ← x=2-13, full-width ROAD (2 cells)
  y=16  │ road         │
  y=15  │  row D ══════│  ← 6 spots, x=5-10
  y=14  │  ── divider ─│
  y=13  │  row C ══════│  ← 6 spots
  y=12  │ central lane │  ← x=4-11 ROAD (2 cells) — COORDINATION BOTTLENECK
  y=11  │              │
  y=10  │  row B ══════│  ← 6 spots
  y= 9  │  ── divider ─│
  y= 8  │  row A ══════│  ← 6 spots
  y= 7  │ bottom access│  ← x=2-13, full-width ROAD (2 cells)
  y= 6  │ road         │
  y= 5  │ bottom queue │  ← x=2-3 and x=12-13 only, ROAD
  y= 4  │              │
  y= 3  │              │
  y= 2  │              │
  y= 1  │  outer wall  │
  y= 0  └──────────────┘
          x=0 1 2 3 4 5…10 11 12 13 14 15
               │perim│div│ spots│div│perim│

Per column:
  x=0-1   : outer wall
  x=2-3   : left perimeter (ROAD, y=2..21)
  x=4     : left interior divider (WALL in spot rows, ROAD in lane/access rows)
  x=5-10  : 6 parking spots (1 cell each)
  x=11    : right interior divider
  x=12-13 : right perimeter
  x=14-15 : outer wall

Cell values:  0=road  1=parking spot  100=wall/impassable
Headings:     0=East  1=North  2=West  3=South
"""

import numpy as np
import os

# ── Grid parameters ────────────────────────────────────────────────────────
W, H = 16, 24
WALL, ROAD, SPOT = 100, 0, 1
COLLISION_THRESH = 100

SPOT_COLS = list(range(5, 11))   # x=5..10 → 6 spots per row

# 4 parking rows (y_start, y_end_exclusive), each 1 cell deep
PARKING_ROWS = [
    (8,  9),   # row A — faces bottom access road (y=6-7) from above
    (10, 11),  # row B — faces central lane (y=11-12) from below
    (13, 14),  # row C — faces central lane (y=11-12) from above
    (15, 16),  # row D — faces top access road (y=16-17) from below
]

BOTTOM_ACCESS = (6,  8)   # y=6-7
CENTRAL_LANE  = (11, 13)  # y=11-12  ← the bottleneck
TOP_ACCESS    = (16, 18)  # y=16-17

# Perimeter queue bands (agents wait here before entering)
TOP_QUEUE_Y    = range(18, 22)   # y=18..21 (4 rows)
BOTTOM_QUEUE_Y = range(2, 6)     # y=2..5  (4 rows)
PERIM_COLS     = [2, 3, 12, 13]  # the 4 perimeter columns


def generate() -> np.ndarray:
    """Return the (W, H) costmap array."""
    g = np.full((W, H), WALL, dtype=int)

    def fill(x1, x2, y1, y2, val):
        g[x1:x2, y1:y2] = val

    # Perimeter roads — run from y=2 to y=21 (between outer walls)
    fill(2,  4,  2, 22, ROAD)   # left  x=2-3
    fill(12, 14, 2, 22, ROAD)   # right x=12-13

    # Bottom access road — full interior width
    fill(2, 14, *BOTTOM_ACCESS, ROAD)

    # Central lane — interior width only (x=4-11); perimeters already ROAD
    fill(4, 12, *CENTRAL_LANE, ROAD)

    # Top access road
    fill(2, 14, *TOP_ACCESS, ROAD)

    # Parking spots (x=5-10; x=4 and x=11 stay WALL as interior dividers)
    for y1, y2 in PARKING_ROWS:
        fill(5, 11, y1, y2, SPOT)

    return g


def write_map(grid: np.ndarray, path: str, agents=()) -> None:
    """Write map file in N/C/A/M format."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write(f'N\n{W},{H}\n')
        f.write(f'C\n{COLLISION_THRESH}\n')
        f.write(f'A\n{len(agents)}\n')
        for (sx, sy, sh), (gx, gy) in agents:
            f.write(f'{sx},{sy},{sh}\n')
            f.write(f'{gx},{gy}\n')
        f.write('M\n')
        for y in range(H):
            f.write(','.join(str(grid[x, y]) for x in range(W)) + '\n')
    print(f'Map written: {path}  ({W}×{H}, {len(agents)} agent(s))')


# ── Spot / start helpers ───────────────────────────────────────────────────

def _bottom_spots():
    """6+6 = 12 spots in rows A and B (bottom half)."""
    spots = []
    for y1, _ in PARKING_ROWS[:2]:   # rows A, B
        for x in SPOT_COLS:
            spots.append((x, y1))
    return spots


def _top_spots():
    """6+6 = 12 spots in rows C and D (top half)."""
    spots = []
    for y1, _ in PARKING_ROWS[2:]:   # rows C, D
        for x in SPOT_COLS:
            spots.append((x, y1))
    return spots


def _top_starts():
    """16 start positions in the top queue, heading South (3)."""
    positions = []
    for y in reversed(TOP_QUEUE_Y):   # y=21 down to y=18 — front-of-queue first
        for x in PERIM_COLS:
            positions.append((x, y, 'S'))
    return positions


def _bottom_starts():
    """16 start positions in the bottom queue, heading North (1)."""
    positions = []
    for y in BOTTOM_QUEUE_Y:          # y=2 up to y=5 — front-of-queue first
        for x in PERIM_COLS:
            positions.append((x, y, 'N'))
    return positions


# ── Agent configurations ───────────────────────────────────────────────────

def all_crossing_agents():
    """
    24 agents: all goals are on the OPPOSITE side from their start.

      Top agents   (12) → bottom spots (rows A & B)
      Bottom agents (12) → top spots   (rows C & D)

    Every agent must traverse the central lane, producing guaranteed
    bidirectional conflicts that require explicit coordination.
    """
    top_st  = _top_starts()[:12]
    bot_st  = _bottom_starts()[:12]
    bot_sp  = _bottom_spots()          # 12 bottom spots
    top_sp  = _top_spots()             # 12 top spots
    agents  = []
    for s, g in zip(top_st, bot_sp):   # top agent → bottom spot
        agents.append((s, g))
    for s, g in zip(bot_st, top_sp):   # bottom agent → top spot
        agents.append((s, g))
    return agents


def random_agents(n: int, seed=None):
    """
    n agents with randomized starts (mix of top and bottom) and random goals,
    still ensuring starts and goals are distinct.

    n must be ≤ 24 (12 per side, 24 total spots).
    """
    import random
    rng = random.Random(seed)

    all_starts = _top_starts() + _bottom_starts()   # 32 total
    all_goals  = _bottom_spots() + _top_spots()     # 24 total

    if n > len(all_starts):
        raise ValueError(f'n={n} exceeds available starts ({len(all_starts)})')
    if n > len(all_goals):
        raise ValueError(f'n={n} exceeds available spots ({len(all_goals)})')

    chosen_starts = rng.sample(all_starts, n)
    chosen_goals  = rng.sample(all_goals,  n)
    return list(zip(chosen_starts, chosen_goals))


# 8-agent example: 4 from each side, all crossing
EXAMPLE_AGENTS = [
    ((2,  21, 'S'), (5,  8)),   # top-left  → spot A-1
    ((12, 21, 'S'), (10, 8)),   # top-right → spot A-6
    ((3,  20, 'S'), (7, 10)),   # top-left  → spot B-3
    ((13, 20, 'S'), (8, 10)),   # top-right → spot B-4
    ((2,  2,  'N'), (5, 13)),   # bot-left  → spot C-1
    ((12, 2,  'N'), (10,13)),   # bot-right → spot C-6
    ((3,  3,  'N'), (7, 15)),   # bot-left  → spot D-3
    ((13, 3,  'N'), (8, 15)),   # bot-right → spot D-4
]


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description='Generate the Gauntlet coordination map.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python map/generate_gauntlet.py                      # 24 agents, all crossing
  python map/generate_gauntlet.py --example            # 8-agent test case
  python map/generate_gauntlet.py --random 12          # 12 random agents
  python map/generate_gauntlet.py --random 16 --seed 7 # reproducible random
""")
    ap.add_argument('--example', action='store_true',
                    help='Write 8-agent example instead of all 24')
    ap.add_argument('--random', metavar='N', type=int, default=None,
                    help='Generate N agents with randomized starts and goals')
    ap.add_argument('--seed', type=int, default=None,
                    help='Random seed for --random')
    args = ap.parse_args()

    grid = generate()

    # Sanity checks
    assert grid[5,  8] == SPOT, "row A spot 1 should be SPOT"
    assert grid[10, 8] == SPOT, "row A spot 6 should be SPOT"
    assert grid[5, 10] == SPOT, "row B spot 1 should be SPOT"
    assert grid[5, 13] == SPOT, "row C spot 1 should be SPOT"
    assert grid[5, 15] == SPOT, "row D spot 1 should be SPOT"
    assert grid[4,  8] == WALL, "x=4 in parking row should be WALL"
    assert grid[4, 11] == ROAD, "x=4 in central lane should be ROAD"
    assert grid[4,  6] == ROAD, "x=4 in bottom access should be ROAD"
    assert grid[2,  5] == ROAD, "left perimeter in queue area should be ROAD"
    assert grid[2, 11] == ROAD, "left perimeter at central lane should be ROAD"
    n_spots = sum(1 for x in range(W) for y in range(H) if grid[x, y] == SPOT)
    assert n_spots == 24, f"expected 24 SPOT cells, got {n_spots}"
    print("All sanity checks passed.")

    if args.random is not None:
        agents = random_agents(args.random, seed=args.seed)
        seed_str = f'  seed={args.seed}' if args.seed is not None else '  (non-deterministic)'
        print(f'Random mode:{seed_str}')
    elif args.example:
        agents = EXAMPLE_AGENTS
    else:
        agents = all_crossing_agents()

    out = os.path.join(os.path.dirname(__file__), 'gauntlet.txt')
    write_map(grid, out, agents)
    print(f'Parking spots: {n_spots}  |  Agents: {len(agents)}')
