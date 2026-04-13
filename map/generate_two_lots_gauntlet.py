#!/usr/bin/env python3
"""
Generate the "Two Lots Gauntlet" — compact 16-agent crossing scenario.

── Why it's harder ────────────────────────────────────────────────────────────
8 agents start queued on the LEFT lot's right perimeter heading into the lot,
with goals in the RIGHT lot.  8 agents do the reverse.  Every agent MUST cross
one of the three roads.

Roads 1 and 2 are only 2 cells wide (x=14-15) — a brutal pinch point.  Two
agents heading in opposite directions can just barely squeeze past each other
(one at y=7, one at y=8), so any group of 3+ competing for the same road must
wait.  Road 3 is a long detour (~30+ extra steps) via the top bypass.

── Layout (30 × 26 grid, y=0 at bottom) ─────────────────────────────────────

  y=25  ████████████████████████████ (outer wall)
  y=24  ████████████████████████████
  y=23  ██─────────────────────────██
  y=22  ██ ═══ Road 3 bypass ══════ ██  (x=2-27, ROAD)
  y=21  ██ ════════════════════════ ██
  y=20  ██[L-perim]██████[R-perim]██   (interior WALL above y=18)
  y=19  same
  y=18  ██ top access lane   │WALL│ top access lane ██
  y=17  same
  y=16  ██ row F (6 spots)   │WALL│ row F           ██
  y=15  ██ == divider ==     │    │ == divider ==   ██
  y=14  ██ row E             ║ R2 ║ row E           ██  ← Road 2 (bottleneck)
  y=13  ██ lane 2 / Road 2   ║════║ lane 2          ██
  y=12  ██ row D             │WALL│ row D           ██
  y=11  ██ == divider ==     │    │ == divider ==   ██
  y=10  ██ row C             │WALL│ row C           ██
  y= 9  ██ lane 1 (internal) │WALL│ lane 1 internal ██
  y= 8  ██ lane 1 / Road 1   ║ R1 ║ lane 1          ██  ← Road 1 (bottleneck)
  y= 7  ██ ═══════════════════════ ██
  y= 6  ██ row B             │WALL│ row B           ██
  y= 5  ██ == divider ==     │    │ == divider ==   ██
  y= 4  ██ row A             │WALL│ row A           ██
  y= 3  ██ bottom access     │WALL│ bottom access   ██
  y= 2  same
  y= 1  ████████████████████████████ (outer wall)
  y= 0  ████████████████████████████

  x=    0 2   4  5      10 11  12 13  14 15  16 17  18  19      24 25  26 27  28 29
            │per│div│ 6sp │div│ perim │ROAD│ perim │div│ 6sp    │div│ perim │wall

Column layout:
  x=0-1   : left outer wall
  x=2-3   : left lot left perimeter (ROAD, y=2..22)
  x=4     : left lot interior divider
  x=5-10  : left lot spots (6 per row)
  x=11    : left lot interior divider
  x=12-13 : left lot right perimeter (ROAD, y=2..22)
  x=14-15 : MIDDLE — 2-cell bottleneck (ROAD only at Road 1 & Road 2 y-ranges)
  x=16-17 : right lot left perimeter (ROAD, y=2..22)
  x=18    : right lot interior divider
  x=19-24 : right lot spots (6 per row)
  x=25    : right lot interior divider
  x=26-27 : right lot right perimeter (ROAD, y=2..22)
  x=28-29 : right outer wall

Row layout:
  y=0-1   : bottom outer wall
  y=2-3   : bottom access road
  y=4     : row A (SPOT)
  y=5     : divider (WALL)
  y=6     : row B (SPOT)
  y=7-8   : lane 1 / Road 1 (ROAD — bottleneck at x=14-15)
  y=9     : lane 1.5 / internal (ROAD in lots only; middle WALL)
  y=10    : row C (SPOT)
  y=11    : divider
  y=12    : row D (SPOT)
  y=13-14 : lane 2 / Road 2 (ROAD — bottleneck at x=14-15)
  y=15    : divider
  y=16    : row E (SPOT)  [note: no row between Road2 and row E]
  y=17-18 : top access road
  y=19-20 : perimeter extension (lot interior WALL)
  y=21-22 : Road 3 bypass (ROAD, x=2-27)
  y=23-25 : top outer wall

Cell values: 0=road  1=parking spot  100=wall
Headings:    0=East  1=North  2=West  3=South
"""

import numpy as np
import os
import random

W, H = 30, 26
WALL, ROAD, SPOT = 100, 0, 1
COLLISION_THRESH = 100

SPOT_COLS_L = list(range(5, 11))    # x=5..10 (6 spots)
SPOT_COLS_R = list(range(19, 25))   # x=19..24 (6 spots)

PARKING_YS = [4, 6, 10, 12, 16]    # 5 parking rows (A, B, C, D, E)

# Road 1: y=7-8 (upper bottleneck)
# Road 2: y=13-14 (lower bottleneck)
# Road 3: y=21-22 (long bypass)


def generate():
    """Return the (W, H) costmap array."""
    g = np.full((W, H), WALL, dtype=int)

    # ── Perimeters (y=2..22) ───────────────────────────────────────────────
    g[2:4,   2:23] = ROAD   # left lot left perim
    g[12:14, 2:23] = ROAD   # left lot right perim
    g[16:18, 2:23] = ROAD   # right lot left perim
    g[26:28, 2:23] = ROAD   # right lot right perim

    # ── Left lot interior roads ────────────────────────────────────────────
    g[4:14, 2:4]   = ROAD   # bottom access
    g[4:14, 7:9]   = ROAD   # lane 1 / Road 1 (left side)
    g[4:14, 9:10]  = ROAD   # extra internal lane row (y=9)
    g[4:14, 13:15] = ROAD   # lane 2 / Road 2 (left side)
    g[4:14, 17:19] = ROAD   # top access

    # ── Right lot interior roads ───────────────────────────────────────────
    g[16:26, 2:4]   = ROAD
    g[16:26, 7:9]   = ROAD
    g[16:26, 9:10]  = ROAD
    g[16:26, 13:15] = ROAD
    g[16:26, 17:19] = ROAD

    # ── Road 1: 2-cell bottleneck at y=7-8 ────────────────────────────────
    g[14:16, 7:9] = ROAD

    # ── Road 2: 2-cell bottleneck at y=13-14 ──────────────────────────────
    g[14:16, 13:15] = ROAD

    # ── Road 3: long bypass at y=21-22, x=2-27 ────────────────────────────
    g[2:28, 21:23] = ROAD

    # ── Parking spots ──────────────────────────────────────────────────────
    for y in PARKING_YS:
        g[5:11,  y] = SPOT   # left lot
        g[19:25, y] = SPOT   # right lot

    return g


def write_map(grid, path, agents=()):
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


# ── Agent configuration ────────────────────────────────────────────────────

def _left_spots():
    return [(x, y) for y in PARKING_YS for x in SPOT_COLS_L]


def _right_spots():
    return [(x, y) for y in PARKING_YS for x in SPOT_COLS_R]


# 16-agent gauntlet: all agents must cross
# Left agents queue on left lot right perim (x=12-13), heading south
# Right agents queue on right lot left perim (x=16-17), heading south
# Queue zone: y=22 down to y=19 (4 rows × 2 cols = 8 per side)

GAUNTLET_AGENTS = [
    # Left lot agents → right lot goals  (must cross Road 1 or 2 or 3)
    ((12, 22, 'S'), (19,  4)),   # L0 → right row A left
    ((13, 22, 'S'), (24,  4)),   # L1 → right row A right
    ((12, 21, 'S'), (19,  6)),   # L2 → right row B left
    ((13, 21, 'S'), (24,  6)),   # L3 → right row B right
    ((12, 20, 'S'), (19, 10)),   # L4 → right row C left
    ((13, 20, 'S'), (24, 10)),   # L5 → right row C right
    ((12, 19, 'S'), (19, 12)),   # L6 → right row D left
    ((13, 19, 'S'), (24, 16)),   # L7 → right row E right

    # Right lot agents → left lot goals  (must cross Road 1 or 2 or 3)
    ((16, 22, 'S'), (10,  4)),   # R0 → left row A right
    ((17, 22, 'S'), ( 5,  4)),   # R1 → left row A left
    ((16, 21, 'S'), (10,  6)),   # R2 → left row B right
    ((17, 21, 'S'), ( 5,  6)),   # R3 → left row B left
    ((16, 20, 'S'), (10, 10)),   # R4 → left row C right
    ((17, 20, 'S'), ( 5, 10)),   # R5 → left row C left
    ((16, 19, 'S'), (10, 12)),   # R6 → left row D right
    ((17, 19, 'S'), ( 5, 16)),   # R7 → left row E left
]


def random_agents(n, seed=None):
    rng = random.Random(seed)
    l_q = [(x, y, 'S') for y in range(22, 18, -1) for x in [12, 13]]
    r_q = [(x, y, 'S') for y in range(22, 18, -1) for x in [16, 17]]
    all_starts = l_q + r_q
    all_goals  = _left_spots() + _right_spots()
    starts = rng.sample(all_starts, n)
    goals  = rng.sample(all_goals,  n)
    return list(zip(starts, goals))


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description='Generate the Two Lots Gauntlet map (16-agent crossing).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python map/generate_two_lots_gauntlet.py           # 16 agents, fixed layout
  python map/generate_two_lots_gauntlet.py --random 16 --seed 7
""")
    ap.add_argument('--random', metavar='N', type=int, default=None,
                    help='N agents with randomized starts/goals (max 16)')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    grid = generate()

    # Sanity checks
    assert grid[5,  4]  == SPOT, "left lot row A leftmost spot"
    assert grid[10, 4]  == SPOT, "left lot row A rightmost spot"
    assert grid[19, 4]  == SPOT, "right lot row A leftmost spot"
    assert grid[24, 4]  == SPOT, "right lot row A rightmost spot"
    assert grid[14, 7]  == ROAD, "Road 1 bottleneck cell"
    assert grid[14, 13] == ROAD, "Road 2 bottleneck cell"
    assert grid[14, 12] == WALL, "middle wall between Road 1 and Road 2"
    assert grid[10, 21] == ROAD, "Road 3 bypass cell"
    assert grid[4,  7]  == ROAD, "left lot lane 1"
    assert grid[4,  4]  == WALL, "interior divider in spot row"
    assert grid[12, 20] == ROAD, "left lot right perim in queue zone"
    assert grid[16, 20] == ROAD, "right lot left perim in queue zone"
    n_spots = sum(1 for x in range(W) for y in range(H) if grid[x, y] == SPOT)
    expected = 2 * len(PARKING_YS) * 6
    assert n_spots == expected, f"spot count: expected {expected}, got {n_spots}"
    print(f'Sanity checks passed.  Spots: {n_spots}')

    if args.random is not None:
        agents = random_agents(args.random, seed=args.seed)
    else:
        agents = GAUNTLET_AGENTS

    out = os.path.join(os.path.dirname(__file__), 'two_lots_gauntlet.txt')
    write_map(grid, out, agents)
    print(f'Agents: {len(agents)}')
