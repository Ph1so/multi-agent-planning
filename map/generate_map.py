#!/usr/bin/env python3
"""
Generate a parking lot map for multi-agent autonomous vehicle planning.

── Design principles ────────────────────────────────────────────────────────
Each grid cell represents exactly one car-length / car-width unit.

  Parking spots : 1 × 1 cell  (one car exactly, no wasted space)
  Driving lanes : 2 cells wide (two cars can pass side by side)
  Spot rows     : 1 cell deep, packed side-by-side with no dividers

── Layout (28 × 64 grid, 0-indexed, y=0 at bottom) ─────────────────────────

  y=63 ┌─────────────────────────┐
  y=62 │  outer wall             │
  y=28 │  [agent queue area]     │  ← perimeter x=2-3 and x=24-25, ROAD
  y=24 │  ─── interior top wall ─│  ← x=4-23 stays WALL above lane 4
  y=22 │──── lane 4 (y=22-23) ──│  ← 2-wide road between row H and entry
  y=21 │  parking row H          │
  y=20 │  ══ divider ══          │
  y=19 │  parking row G          │
  y=17 │──── lane 3 (y=17-18) ──│
  y=16 │  parking row F          │
  y=15 │  ══ divider ══          │
  y=14 │  parking row E          │
  y=12 │──── lane 2 (y=12-13) ──│
  y=11 │  parking row D          │
  y=10 │  ══ divider ══          │
  y= 9 │  parking row C          │
  y= 7 │──── lane 1 (y=7-8) ────│
  y= 6 │  parking row B          │
  y= 5 │  ══ divider ══          │
  y= 4 │  parking row A          │
  y= 2 │──── access road (y=2-3) │
  y= 0 └─────────────────────────┘
         x=0  x=2-3      x=4  x=5…22     x=23  x=24-25   x=26-27
              │ perim │  div  │18 spots│  div  │ perim  │  wall

  Per column (x-axis):
    x=0-1   : outer wall
    x=2-3   : left  perimeter road  (vertical connector, always ROAD y=2..63)
    x=4     : left  interior divider (WALL in spot rows, ROAD in lane rows)
    x=5-22  : 18 parking spots (1 cell each, packed with no dividers)
    x=23    : right interior divider (same rule as x=4)
    x=24-25 : right perimeter road
    x=26-27 : outer wall

  Per row (y-axis):
    y=0-1   : bottom outer wall
    y=2-3   : access road (connects both perimeters; feeds row A)
    y=4     : parking row A (1 cell deep)
    y=5     : divider wall (between back-to-back rows A and B)
    y=6     : parking row B
    y=7-8   : lane 1   (2 cells wide)
    y=9     : parking row C
    y=10    : divider wall
    y=11    : parking row D
    y=12-13 : lane 2
    y=14    : parking row E
    y=15    : divider wall
    y=16    : parking row F
    y=17-18 : lane 3
    y=19    : parking row G
    y=20    : divider wall
    y=21    : parking row H
    y=22-23 : lane 4   (top of interior; entry from perimeter)
    y=24+   : interior stays WALL (perimeter roads pass through freely)
    y=62-63 : top outer wall

Cell values:
  0   = road  (driveable, free)
  1   = parking spot  (driveable, designated)
  100 = wall / obstacle  (impassable)

── State representation ─────────────────────────────────────────────────────
Planner state is (x, y, heading).  Headings:
  0 = East (→)   1 = North (↑)   2 = West (←)   3 = South (↓)
"""

import numpy as np
import os

# ── Grid parameters ────────────────────────────────────────────────────────
W, H = 28, 64          # x (width), y (height)
WALL, ROAD, SPOT = 100, 0, 1
COLLISION_THRESH = 100

# 18 spots per row, each 1 cell wide, packed with no dividers between them
SPOT_STARTS = list(range(5, 23))   # [5, 6, 7, …, 22]  → 18 spots

# 8 parking rows (y_start, y_end_exclusive), each 1 cell deep
PARKING_ROWS = [
    (4,  5),   # row A – faces access road (y=2-3)
    (6,  7),   # row B – faces lane 1 from below
    (9,  10),  # row C – faces lane 1 from above
    (11, 12),  # row D – faces lane 2 from below
    (14, 15),  # row E – faces lane 2 from above
    (16, 17),  # row F – faces lane 3 from below
    (19, 20),  # row G – faces lane 3 from above
    (21, 22),  # row H – faces lane 4 from below
]

# 4 driving lanes, each 2 cells wide (two cars side by side)
LANE_ROWS = [(7, 9), (12, 14), (17, 19), (22, 24)]

# Divider walls between back-to-back parking sections (stay WALL from init):
#   y=5   (between rows A and B)
#   y=10  (between rows C and D)
#   y=15  (between rows E and F)
#   y=20  (between rows G and H)
# Interior top wall (stays WALL): y=24-63 at x=4-23


def generate() -> np.ndarray:
    """Return the (W, H) costmap array."""
    g = np.full((W, H), WALL, dtype=int)

    def fill(x1, x2, y1, y2, val):
        g[x1:x2, y1:y2] = val

    # Perimeter roads — run full interior height
    fill(2,  4,  2, H, ROAD)    # left  x=2-3
    fill(24, 26, 2, H, ROAD)    # right x=24-25

    # Access road at bottom of interior (connects left & right perimeter)
    fill(4, 24, 2, 4, ROAD)     # y=2-3

    # Horizontal driving lanes — span full interior width
    for y1, y2 in LANE_ROWS:
        fill(4, 24, y1, y2, ROAD)

    # Parking spots — 1 cell each, packed side by side (x=5-22)
    # x=4 and x=23 stay WALL in spot rows (interior dividers)
    for y1, y2 in PARKING_ROWS:
        fill(5, 23, y1, y2, SPOT)

    return g


def write_map(grid: np.ndarray, path: str, agents=()) -> None:
    """
    Write map file.

    Agent tuple format:  ((sx, sy, heading), (gx, gy))
      heading: 'N', 'S', 'E', 'W'  or  0/1/2/3

    File format:
      N\\n<x_size>,<y_size>
      C\\n<collision_threshold>
      A\\n<num_agents>
      <start_x>,<start_y>,<heading>   ← one line per agent start
      <goal_x>,<goal_y>               ← one line per agent goal
      M\\n
      <row y=0 : x_0,x_1,…,x_{W-1}>
      …
      <row y=H-1>
    """
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


# ── Agent configurations ────────────────────────────────────────────────────

def _all_spot_centers():
    """Return list of all 144 parking spot centers (gx, gy)."""
    spots = []
    for y1, _ in PARKING_ROWS:       # each row is 1 cell deep; center = y1
        for sx in SPOT_STARTS:        # each spot is 1 cell wide; center = sx
            spots.append((sx, y1))
    return spots


def _all_start_positions():
    """
    Return all 144 valid start positions in the perimeter roads.
    4 columns × 36 y-values (y=63 down to y=28), all confirmed ROAD cells.
    """
    entry_cols = [2, 3, 24, 25]
    positions = []
    for y in range(63, 27, -1):        # y=63 … y=28  (36 rows × 4 cols = 144)
        for col in entry_cols:
            positions.append((col, y, 'N'))
    return positions


def all_parking_agents():
    """
    Return one agent per parking spot (144 total), deterministic ordering.

    Goals:  every spot center, ordered row A→H, left→right.
    Starts: agents queue along the four perimeter columns
            (x=2, 3, 24, 25) stepping back from y=63.
    """
    goals  = _all_spot_centers()
    starts = _all_start_positions()
    return [(starts[i], goals[i]) for i in range(len(goals))]


def random_agents(n: int, seed=None):
    """
    Return n agents with randomized start positions and goal assignments.

    Parameters
    ----------
    n    : number of agents (1 – 144)
    seed : integer random seed for reproducibility (None = non-deterministic)

    Starts are drawn without replacement from the 144 valid perimeter-road
    cells.  Goals are drawn without replacement from the 144 spot centers.
    Neither set overlaps (perimeter vs. interior are disjoint).
    """
    import random
    rng = random.Random(seed)

    all_starts = _all_start_positions()
    all_goals  = _all_spot_centers()

    if n > len(all_starts):
        raise ValueError(f'n={n} exceeds available start positions ({len(all_starts)})')
    if n > len(all_goals):
        raise ValueError(f'n={n} exceeds available parking spots ({len(all_goals)})')

    chosen_starts = rng.sample(all_starts, n)
    chosen_goals  = rng.sample(all_goals,  n)
    return list(zip(chosen_starts, chosen_goals))


# Small 4-agent example for quick testing
EXAMPLE_AGENTS = [
    ((2,  63, 'N'), (5,   4)),   # Agent 0: left entry  → spot A-1
    ((3,  63, 'N'), (13,  6)),   # Agent 1: left entry  → spot B-9
    ((24, 63, 'N'), (22,  4)),   # Agent 2: right entry → spot A-18
    ((25, 63, 'N'), (13, 21)),   # Agent 3: right entry → spot H-9
]


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description='Generate the parking lot map file.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python map/generate_map.py                        # 144 agents, deterministic
  python map/generate_map.py --example              # 4-agent test case
  python map/generate_map.py --random 8             # 8 agents, random
  python map/generate_map.py --random 16 --seed 42  # reproducible random
""")
    ap.add_argument('--example', action='store_true',
                    help='Write 4-agent example instead of all 144')
    ap.add_argument('--random', metavar='N', type=int, default=None,
                    help='Generate N agents with randomized starts and goals')
    ap.add_argument('--seed', type=int, default=None,
                    help='Random seed for --random (omit for non-deterministic)')
    args = ap.parse_args()

    grid = generate()

    # Sanity checks
    assert grid[5,  4] == SPOT, "spot A-1 (x=5,y=4) should be SPOT"
    assert grid[13, 6] == SPOT, "spot B-9 (x=13,y=6) should be SPOT"
    assert grid[5,  6] == SPOT, "spot B-1 (x=5,y=6) should be SPOT"
    assert grid[5,  7] == ROAD, "y=7 is bottom of lane 1 (ROAD)"
    assert grid[5,  8] == ROAD, "y=8 is top of lane 1 (ROAD)"
    assert grid[5,  9] == SPOT, "y=9 is row C (SPOT)"
    assert grid[4,  7] == ROAD, "x=4 in lane row should be ROAD"
    assert grid[4,  4] == WALL, "x=4 in parking row should be WALL"
    assert grid[23, 7] == ROAD, "x=23 in lane row should be ROAD"
    assert grid[23, 4] == WALL, "x=23 in parking row should be WALL"
    assert grid[2, 40] == ROAD, "left perimeter should be ROAD through all rows"
    assert grid[2, 63] == ROAD, "left entry cell should be ROAD"
    assert grid[24,40] == ROAD, "right perimeter should be ROAD"
    for y1, y2 in LANE_ROWS:
        w = sum(1 for y in range(y1, y2) if grid[10, y] == ROAD)
        assert w == 2, f"lane y={y1}-{y2-1} should be 2 cells wide, got {w}"
    n_spots_found = sum(1 for x in range(W) for y in range(H) if grid[x, y] == SPOT)
    assert n_spots_found == 144, f"expected 144 SPOT cells, got {n_spots_found}"
    print("All sanity checks passed.")

    if args.random is not None:
        agents = random_agents(args.random, seed=args.seed)
        seed_str = f'  seed={args.seed}' if args.seed is not None else '  (non-deterministic)'
        print(f'Random mode:{seed_str}')
    elif args.example:
        agents = EXAMPLE_AGENTS
    else:
        agents = all_parking_agents()

    out = os.path.join(os.path.dirname(__file__), 'parking_lot.txt')
    write_map(grid, out, agents)
    n_spots = len(PARKING_ROWS) * len(SPOT_STARTS)
    print(f'Parking spots: {n_spots}  |  Agents: {len(agents)}')
