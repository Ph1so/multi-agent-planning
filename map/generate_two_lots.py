#!/usr/bin/env python3
"""
Generate the "Two Lots" map — two parking lots connected by 3 roads.

── Design ─────────────────────────────────────────────────────────────────────
Two mirror-image parking lots sit side-by-side separated by a 4-cell wall.
Three roads connect them:

  Road 1 (short, upper) — y=7-8,   x=22-25 — aligns with lots' lane 1
  Road 2 (short, lower) — y=17-18, x=22-25 — aligns with lots' lane 3
  Road 3 (long bypass)  — y=33-34, x=2-45  — detour via top perimeter

Roads 1 and 2 are narrow (2 cells tall × 4 cells wide).  Agents travelling
in opposite directions must negotiate passage, forcing waits.  Road 3 is
~30+ cells longer but avoids the bottleneck entirely.

── Layout (48 × 52 grid, y=0 at bottom) ─────────────────────────────────────

  y=51  ████████████████████████████████████████████████ (outer wall)
  y=50  ████████████████████████████████████████████████
  y=49  ██                                            ██ ╗
  y=48  ██      B I G   S T A G I N G   A R E A      ██ ║
  ...   ██          (x=2-45 fully open, 660 cells)   ██ ║ up to 660
  y=36  ██                                            ██ ║ start positions
  y=35  ██                                            ██ ╝
  y=34  ██ ════════════ Road 3 bypass (x=2-45) ══════ ██  ← lot entry level
  y=33  ██ ═══════════════════════════════════════════ ██
  y=32  ██[left perim]████████████████████[right perim]██
  ...   (interior stays WALL above y=23; perims continue)
  y=23  ██L-perim│top access lane    │L-perim│WALL│R-perim│top access lane│R-perim██
  y=22  (same)
  y=21  ██      │ row H (14 spots)   │      │WALL│       │ row H          │      ██
  y=20  ██      │ ══ divider ══      │      │    │       │ ══ divider ══  │      ██
  y=19  ██      │ row G              │      │    │       │ row G          │      ██
  y=18  ██      ║ lane3 / Road 2 ════════════════════════ lane3           ║      ██
  y=17  ██      ║══════════════════════════════════════════════════════════      ██
  y=16  ██      │ row F              │      │WALL│       │ row F          │      ██
  y=15  ██      │ ══ divider ══      │      │    │       │ ══ divider ══  │      ██
  y=14  ██      │ row E              │      │    │       │ row E          │      ██
  y=13  ██      │ lane 2 (no road)   │      │WALL│       │ lane 2         │      ██
  y=12  (same)
  y=11  ██      │ row D              │      │    │       │ row D          │      ██
  y=10  ██      │ ══ divider ══      │      │    │       │ ══ divider ══  │      ██
  y= 9  ██      │ row C              │      │    │       │ row C          │      ██
  y= 8  ██      ║ lane1 / Road 1 ════════════════════════ lane1           ║      ██
  y= 7  ██      ║══════════════════════════════════════════════════════════      ██
  y= 6  ██      │ row B              │      │WALL│       │ row B          │      ██
  y= 5  ██      │ ══ divider ══      │      │    │       │ ══ divider ══  │      ██
  y= 4  ██      │ row A              │      │    │       │ row A          │      ██
  y= 3  ██      │ bottom access road │      │WALL│       │ bottom access  │      ██
  y= 2  (same)
  y= 1  ████████████████████████████████████████████████ (outer wall)
  y= 0  ████████████████████████████████████████████████

  x=    0 2   4  5              18 19  20 21  22   25  26 27  28  29            42 43  44 45  46 47
           │per│div│  14 spots  │div│ perim │ MIDDLE │ perim │div│  14 spots   │div│ perim │wall

Column layout:
  x=0-1   : left outer wall
  x=2-3   : left lot left perimeter (ROAD, y=2..49)
  x=4     : left lot interior divider (WALL in spot rows, ROAD elsewhere)
  x=5-18  : left lot spots (14 per row)
  x=19    : left lot interior divider
  x=20-21 : left lot right perimeter (ROAD, y=2..49)
  x=22-25 : MIDDLE WALL — Road 1 open at y=7-8, Road 2 at y=17-18,
             fully open in staging area (y=35-49)
  x=26-27 : right lot left perimeter (ROAD, y=2..49)
  x=28    : right lot interior divider
  x=29-42 : right lot spots (14 per row)
  x=43    : right lot interior divider
  x=44-45 : right lot right perimeter (ROAD, y=2..49)
  x=46-47 : right outer wall

Row layout (shared by both lots):
  y=0-1   : bottom outer wall
  y=2-3   : bottom access road
  y=4     : row A (SPOT)
  y=5     : divider (WALL)
  y=6     : row B (SPOT)
  y=7-8   : lane 1 / Road 1 (ROAD across all — bottleneck in middle)
  y=9     : row C (SPOT)
  y=10    : divider
  y=11    : row D (SPOT)
  y=12-13 : lane 2 (ROAD in lots only — middle stays WALL)
  y=14    : row E (SPOT)
  y=15    : divider
  y=16    : row F (SPOT)
  y=17-18 : lane 3 / Road 2 (ROAD across all — bottleneck in middle)
  y=19    : row G (SPOT)
  y=20    : divider
  y=21    : row H (SPOT)
  y=22-23 : top access lane
  y=24-32 : perimeters only (lot interior stays WALL)
  y=33-34 : Road 3 bypass (ROAD, x=2-45) — lot entry/exit level
  y=35-49 : STAGING AREA (x=2-45 fully open, 660 start positions)
  y=50-51 : top outer wall

Cell values: 0=road  1=parking spot  100=wall
Headings:    0=East  1=North  2=West  3=South
"""

import numpy as np
import os
import random

W, H = 48, 52
WALL, ROAD, SPOT = 100, 0, 1
COLLISION_THRESH = 100

# ── Column constants ───────────────────────────────────────────────────────
L_PERIM_L   = (2,  4)    # left lot left perimeter
L_INT       = (4, 22)    # left lot full interior
L_SPOTS     = (5, 19)    # left lot spot columns
L_PERIM_R   = (20, 22)   # left lot right perimeter

MIDDLE      = (22, 26)   # separator wall (Road 1 & 2 punch through here)

R_PERIM_L   = (26, 28)   # right lot left perimeter
R_INT       = (28, 44)   # right lot full interior
R_SPOTS     = (29, 43)   # right lot spot columns
R_PERIM_R   = (44, 46)   # right lot right perimeter

SPOT_COLS_L = list(range(5,  19))   # 14 spots
SPOT_COLS_R = list(range(29, 43))   # 14 spots

# ── Row constants ──────────────────────────────────────────────────────────
PARKING_YS  = [4, 6, 9, 11, 14, 16, 19, 21]   # 8 parking rows

ROAD1_Y     = (7, 9)     # lane 1 / Road 1
ROAD2_Y     = (17, 19)   # lane 3 / Road 2
ROAD3_Y     = (33, 35)   # lot entry/exit level (y=33-34)
STAGING_Y   = (35, 50)   # big open staging area (y=35-49)

PERIM_Y     = (2, 50)    # perimeters span y=2..49


def generate():
    """Return the (W, H) costmap array."""
    g = np.full((W, H), WALL, dtype=int)

    # ── Perimeter columns (run full height including staging area) ─────────
    g[2:4,   2:50] = ROAD   # left lot: left perim
    g[20:22, 2:50] = ROAD   # left lot: right perim
    g[26:28, 2:50] = ROAD   # right lot: left perim
    g[44:46, 2:50] = ROAD   # right lot: right perim

    # ── Left lot interior roads ────────────────────────────────────────────
    g[4:22, 2:4]   = ROAD   # bottom access
    g[4:22, 7:9]   = ROAD   # lane 1
    g[4:22, 12:14] = ROAD   # lane 2
    g[4:22, 17:19] = ROAD   # lane 3
    g[4:22, 22:24] = ROAD   # top access

    # ── Right lot interior roads (mirror) ──────────────────────────────────
    g[28:44, 2:4]   = ROAD
    g[28:44, 7:9]   = ROAD
    g[28:44, 12:14] = ROAD
    g[28:44, 17:19] = ROAD
    g[28:44, 22:24] = ROAD

    # ── Road 1: middle open at y=7-8 ──────────────────────────────────────
    g[22:26, 7:9] = ROAD

    # ── Road 2: middle open at y=17-18 ────────────────────────────────────
    g[22:26, 17:19] = ROAD

    # ── Road 3: lot entry/exit level y=33-34, x=2-45 ─────────────────────
    g[2:46, 33:35] = ROAD

    # ── Staging area: y=35-49, x=2-45, fully open ─────────────────────────
    g[2:46, 35:50] = ROAD

    # ── Parking spots ──────────────────────────────────────────────────────
    for y in PARKING_YS:
        g[5:19,  y] = SPOT   # left lot
        g[29:43, y] = SPOT   # right lot

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


# ── Agent helpers ──────────────────────────────────────────────────────────

def _left_spots():
    return [(x, y) for y in PARKING_YS for x in SPOT_COLS_L]


def _right_spots():
    return [(x, y) for y in PARKING_YS for x in SPOT_COLS_R]


def _all_starts():
    """
    All valid start positions in the staging area (y=35-49, x=2-45).
    15 rows × 44 cols = 660 positions — enough for all 224 agents.
    Agents face South to head into the lots.
    """
    pos = []
    for y in range(49, 34, -1):        # y=49 down to y=35 (front-first)
        for x in range(2, 46):         # x=2..45
            pos.append((x, y, 'S'))
    return pos  # 660 positions


def _left_queue(n):
    """n starts on the left side of the staging area (x=2-21), heading south."""
    pos = [(x, y, 'S') for y in range(49, 34, -1) for x in range(2, 22)]
    return pos[:n]   # up to 15×20 = 300


def _right_queue(n):
    """n starts on the right side of the staging area (x=26-45), heading south."""
    pos = [(x, y, 'S') for y in range(49, 34, -1) for x in range(26, 46)]
    return pos[:n]   # up to 15×20 = 300


def crossing_agents(n_per_side=8, seed=42):
    """
    n_per_side agents from the left lot → right lot spots,
    n_per_side agents from the right lot → left lot spots.
    All agents must cross through one of the three roads.
    """
    rng = random.Random(seed)
    l_starts = _left_queue(n_per_side)
    r_starts = _right_queue(n_per_side)
    l_goals  = rng.sample(_left_spots(),  n_per_side)
    r_goals  = rng.sample(_right_spots(), n_per_side)

    agents = []
    for s, g in zip(l_starts, r_goals):   # left start → right goal
        agents.append((s, g))
    for s, g in zip(r_starts, l_goals):   # right start → left goal
        agents.append((s, g))
    return agents


def random_agents(n, seed=None):
    """
    n agents with random starts drawn from all perimeter columns and random
    goals drawn from all spots.  Supports up to 224 agents (all spots filled).
    """
    rng = random.Random(seed)
    all_starts = _all_starts()
    all_goals  = _left_spots() + _right_spots()
    if n > len(all_goals):
        raise ValueError(f'n={n} exceeds available spots ({len(all_goals)})')
    starts = rng.sample(all_starts, n)
    goals  = rng.sample(all_goals,  n)
    return list(zip(starts, goals))


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description='Generate the Two Lots coordination map.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python map/generate_two_lots.py                       # 16 agents crossing
  python map/generate_two_lots.py --n 12                # 24 agents crossing
  python map/generate_two_lots.py --random 20 --seed 7  # random 20 agents
""")
    ap.add_argument('--n',      type=int, default=8,
                    help='Agents per side for crossing scenario (default 8)')
    ap.add_argument('--random', metavar='N', type=int, default=None,
                    help='Total agents, randomized starts and goals')
    ap.add_argument('--seed',   type=int, default=42)
    args = ap.parse_args()

    grid = generate()

    # Sanity checks
    assert grid[5,  4]  == SPOT, "left lot row A leftmost spot"
    assert grid[18, 4]  == SPOT, "left lot row A rightmost spot"
    assert grid[29, 4]  == SPOT, "right lot row A leftmost spot"
    assert grid[42, 4]  == SPOT, "right lot row A rightmost spot"
    assert grid[22, 7]  == ROAD, "Road 1: middle cell"
    assert grid[22, 17] == ROAD, "Road 2: middle cell"
    assert grid[22, 12] == WALL, "middle wall at lane 2 (no road)"
    assert grid[10, 33] == ROAD, "Road 3: bypass cell"
    assert grid[4,  7]  == ROAD, "left lot lane 1 interior"
    assert grid[4,  4]  == WALL, "interior divider in spot row"
    assert grid[20, 45] == ROAD, "left lot right perim in staging area"
    assert grid[26, 45] == ROAD, "right lot left perim in staging area"
    assert grid[10, 45] == ROAD, "staging area interior cell"
    assert grid[10, 50] == WALL, "top outer wall"
    n_spots = sum(1 for x in range(W) for y in range(H) if grid[x, y] == SPOT)
    assert n_spots == 2 * len(PARKING_YS) * 14, f"spot count mismatch: {n_spots}"
    print(f'Sanity checks passed.  Spots: {n_spots}')

    if args.random is not None:
        agents = random_agents(args.random, seed=args.seed)
    else:
        agents = crossing_agents(n_per_side=args.n, seed=args.seed)

    out = os.path.join(os.path.dirname(__file__), 'two_lots.txt')
    write_map(grid, out, agents)
