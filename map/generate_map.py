#!/usr/bin/env python3
"""
Generate a parking lot map for multi-agent autonomous vehicle planning.

── Why 3-cell lanes? ────────────────────────────────────────────────────────
Cars have a heading (θ) and can only move forward, backward, or turn.
A 90° turn while moving requires lateral clearance ≥ the turning radius.
With 2-cell lanes a turn-in-place model barely fits; any coupled
turn+move primitive (Dubins / Reeds-Shepp style) needs 3 cells to
complete a 90° sweep without clipping the lane boundary.

── Layout (80 × 64 grid, 0-indexed, y=0 at bottom) ────────────────────────

  y=63 ┌─────────────────────────────────────────────────────────────────┐
  y=62 │  outer wall                                                     │
  y=57 │  [entry L]  bottom wall (WALL at x=4-75)  [entry R]            │
  y=54 │──── lane 4 (y=54-56, 3 cells) ────────────────────────────────│
  y=50 │  parking row H (faces lane 4)                                   │
  y=48 │  ══ divider ══                                                  │
  y=44 │  parking row G (faces lane 3)                                   │
  y=41 │──── lane 3 (y=41-43, 3 cells) ────────────────────────────────│
  y=37 │  parking row F                                                  │
  y=35 │  ══ divider ══                                                  │
  y=31 │  parking row E                                                  │
  y=28 │──── lane 2 (y=28-30, 3 cells) ────────────────────────────────│
  y=24 │  parking row D                                                  │
  y=22 │  ══ divider ══                                                  │
  y=18 │  parking row C (faces lane 1)                                   │
  y=15 │──── lane 1 (y=15-17, 3 cells) ────────────────────────────────│
  y=11 │  parking row B (faces lane 1)                                   │
  y= 9 │  ══ divider ══                                                  │
  y= 5 │  parking row A (faces top road)                                 │
  y= 2 │──── top access road (y=2-4, 3 cells) ────────────────────────│
  y= 0 └─────────────────────────────────────────────────────────────────┘
         x=0  x=2  x=4                              x=75  x=77  x=79
              │perim│                                     │perim│

  Per column (x-axis):
    x=0-1   : outer wall
    x=2-3   : left  perimeter road  (vertical connector)
    x=4     : wall divider  (separates perimeter from spot columns)
    x=5-7   : spot 1    x=8: divider    x=9-11: spot 2  …
    x=73-75 : spot 18
    x=76-77 : right perimeter road
    x=78-79 : outer wall

Cell values:
  0   = road  (driveable, free)
  1   = parking spot  (driveable, designated)
  100 = wall / obstacle  (impassable)

── State representation ────────────────────────────────────────────────────
Because cars can only move forward/backward or turn, the planner state
must be (x, y, heading) — not just (x, y).  The heading is stored as:
  0 = East  (→)   1 = North  (↑)   2 = West  (←)   3 = South  (↓)
"""

import numpy as np
import os

# ── Grid parameters ────────────────────────────────────────────────────────
W, H = 80, 64          # x (width), y (height)
WALL, ROAD, SPOT = 100, 0, 1
COLLISION_THRESH = 100

# 18 spots per row; each 3 cells wide with 1-cell wall dividers between
# x=4: left wall divider | x=5-7: spot 1 | x=8: divider | x=9-11: spot 2 | …
SPOT_STARTS = list(range(5, 76, 4))   # [5, 9, 13, …, 73]  → 18 spots

# 8 parking rows (y_start, y_end exclusive)
# Each row is 4 cells deep — enough for both nose-in and back-in parking.
PARKING_ROWS = [
    (5,  9),   # row A – faces top access road (y=2-4)
    (11, 15),  # row B – faces lane 1 from below
    (18, 22),  # row C – faces lane 1 from above
    (24, 28),  # row D – faces lane 2 from below
    (31, 35),  # row E – faces lane 2 from above
    (37, 41),  # row F – faces lane 3 from below
    (44, 48),  # row G – faces lane 3 from above
    (50, 54),  # row H – faces lane 4 from below
]

# 4 driving lanes, each 3 cells wide
# Why 3 cells: minimum clearance for a 90° coupled turn+move primitive
# with turning radius = 1 cell.  2-cell lanes only support turn-in-place.
LANE_ROWS = [(15, 18), (28, 31), (41, 44), (54, 57)]

# Divider walls between back-to-back parking sections (stay WALL from init):
#   y= 9-10  (between rows A and B)
#   y=22-23  (between rows C and D)
#   y=35-36  (between rows E and F)
#   y=48-49  (between rows G and H)
# Bottom wall interior (stay WALL): y=57-63 at x=4-75


def generate() -> np.ndarray:
    """Return the (W, H) costmap array."""
    g = np.full((W, H), WALL, dtype=int)

    def fill(x1, x2, y1, y2, val):
        g[x1:x2, y1:y2] = val

    # Perimeter roads — run full interior height + through outer wall for entry
    fill(2,  4,  2, H, ROAD)    # left  x=2-3  (includes exit gap y=57-63)
    fill(76, 78, 2, H, ROAD)    # right x=76-77

    # Top access road (connects left & right perimeter, feeds row A)
    fill(4, 76, 2, 5, ROAD)     # y=2-4

    # Horizontal driving lanes — span full interior width
    for y1, y2 in LANE_ROWS:
        fill(4, 76, y1, y2, ROAD)

    # Parking spots (column dividers x=4,8,12,…,76 stay WALL from init)
    for y1, y2 in PARKING_ROWS:
        for sx in SPOT_STARTS:
            fill(sx, sx + 3, y1, y2, SPOT)

    return g


def write_map(grid: np.ndarray, path: str, agents=()) -> None:
    """
    Write map file.

    Agent tuple format:  ((sx, sy, heading), (gx, gy))
      heading: 'N', 'S', 'E', 'W'  or  0/1/2/3
      goal heading is optional — omit if any final heading is acceptable

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

def all_parking_agents():
    """
    Return one agent per parking spot (144 total).

    Goals: every spot center, ordered row A→H, left→right.
    Starts: agents queue along the four perimeter entry columns
            (x=2, 3, 76, 77) in groups of 4, stepping back from y=63.

        group 0  agents 0-3   → y=63  (at the lot entrance)
        group 1  agents 4-7   → y=62
        …
        group 35 agents 140-143 → y=28   (furthest back in queue)

    All start cells are valid ROAD cells in the perimeter roads.
    """
    # All 144 spot centers (row-major: A→H, spot 1→18)
    goals = []
    for y1, y2 in PARKING_ROWS:
        yc = (y1 + y2 - 1) // 2      # integer center of 4-deep row
        for sx in SPOT_STARTS:
            goals.append((sx + 1, yc))  # center of 3-wide spot

    # 4 entry columns; agents queue backwards from y=63
    entry_cols = [2, 3, 76, 77]
    agents = []
    for i, goal in enumerate(goals):
        col = entry_cols[i % 4]
        y   = 63 - (i // 4)
        agents.append(((col, y, 'N'), goal))
    return agents


# Small 4-agent example kept for quick testing
EXAMPLE_AGENTS = [
    ((2,  63, 'N'), (6,   6)),   # Agent 0: left entry  → spot A-1
    ((3,  63, 'N'), (38, 12)),   # Agent 1: left entry  → spot B-9
    ((76, 63, 'N'), (74,  6)),   # Agent 2: right entry → spot A-18
    ((77, 63, 'N'), (38, 51)),   # Agent 3: right entry → spot H-9
]


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--example', action='store_true',
                    help='Write 4-agent example instead of all 144')
    args = ap.parse_args()

    grid = generate()

    # Sanity checks
    assert grid[6,  6] == SPOT, "spot A-1 center should be SPOT"
    assert grid[38,12] == SPOT, "spot B-9 center should be SPOT"
    assert grid[6, 14] == SPOT, "y=14 is top of row B (SPOT)"
    assert grid[6, 15] == ROAD, "y=15 is bottom of lane 1 (ROAD)"
    assert grid[6, 17] == ROAD, "y=17 is top of lane 1 (ROAD)"
    assert grid[6, 18] == SPOT, "y=18 is bottom of row C (SPOT)"
    assert grid[4, 15] == ROAD, "x=4 in lane row should be ROAD"
    assert grid[4, 14] == WALL, "x=4 in parking row should be WALL"
    assert grid[2, 40] == ROAD, "left perimeter should be ROAD through parking rows"
    assert grid[2, 63] == ROAD, "left entry cell should be ROAD"
    for y1, y2 in LANE_ROWS:
        w = sum(1 for y in range(y1, y2) if grid[20, y] == ROAD)
        assert w == 3, f"lane y={y1}-{y2-1} should be 3 wide, got {w}"
    print("All sanity checks passed.")

    agents = EXAMPLE_AGENTS if args.example else all_parking_agents()
    out = os.path.join(os.path.dirname(__file__), 'parking_lot.txt')
    write_map(grid, out, agents)
    n_spots = len(PARKING_ROWS) * len(SPOT_STARTS)
    print(f'Parking spots: {n_spots}  |  Agents: {len(agents)}')
