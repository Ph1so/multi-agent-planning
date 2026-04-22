#!/usr/bin/env python3
"""
Trajectory checker for multi-agent parking lot planning.

Verifies:
  1. Vertex conflicts   — no two agents on the same cell at the same timestep
  2. Edge (swap) conflicts — no two agents swapping cells between consecutive steps
  3. Wall collisions    — no agent enters a cell with value >= collision_threshold
  4. Valid moves        — each step is reachable by a legal action
                          (forward, backward, 2-step turn-left,
                           2-step turn-right, wait)
  5. Goal pose          — each recorded trajectory ends at its assigned goal,
                          facing the separator wall when required

Usage:
  python checker.py map/parking_lot.txt output/trajectories.txt
  python checker.py map/parking_lot.txt output/trajectories.txt --verbose
"""

import argparse
import sys
from collections import defaultdict

# heading → (dx, dy)
DX = {0: 1, 1: 0, 2: -1, 3: 0}
DY = {0: 0, 1: 1, 2: 0, 3: -1}


def _parse_heading(s):
    if s is None:
        return None
    mapping = {'E': 0, 'N': 1, 'W': 2, 'S': 3,
               '0': 0, '1': 1, '2': 2, '3': 3}
    return mapping.get(str(s).strip().upper())


# ── Parsers (minimal, no numpy dependency) ───────────────────────────────────

def parse_map(path):
    with open(path) as f:
        lines = [l.rstrip() for l in f if l.strip()]
    i = 0
    result = {}
    while i < len(lines):
        hdr = lines[i].strip(); i += 1
        if hdr == 'N':
            w, h = map(int, lines[i].split(',')); i += 1
            result['W'], result['H'] = w, h
        elif hdr == 'C':
            result['thresh'] = int(lines[i]); i += 1
        elif hdr == 'A':
            n = int(lines[i]); i += 1
            result['num_agents'] = n
            goals = []
            for _ in range(n):
                i += 1   # skip start line
                goal_parts = lines[i].split(','); i += 1
                gx, gy = int(goal_parts[0]), int(goal_parts[1])
                gh = _parse_heading(goal_parts[2]) if len(goal_parts) > 2 else None
                goals.append((gx, gy, gh))
            result['agent_goals'] = goals
        elif hdr == 'M':
            grid = {}
            for y in range(result['H']):
                vals = list(map(int, lines[i].split(','))); i += 1
                for x, v in enumerate(vals):
                    grid[(x, y)] = v
            result['grid'] = grid
    return result


def parse_trajectories(path):
    """
    Returns dict: agent_id -> list of (t, x, y, h) sorted by t.
    Fractional timesteps are rounded to the nearest integer.
    """
    raw = defaultdict(list)
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 4:
                continue
            aid = int(parts[0])
            t   = int(round(float(parts[1])))
            x   = int(round(float(parts[2])))
            y   = int(round(float(parts[3])))
            h   = int(parts[4]) if len(parts) > 4 else -1
            raw[aid].append((t, x, y, h))
    return {aid: sorted(steps) for aid, steps in raw.items()}


# ── Checks ───────────────────────────────────────────────────────────────────

def _build_pos_table(trajs):
    """
    Build a full position table: pos[aid][t] = (x, y) for t in 0..t_max.
    Agents stay at their last recorded position after their trajectory ends
    (they have parked / are waiting at start).
    """
    t_max = max(t for steps in trajs.values() for t, *_ in steps)

    pos = {}
    for aid, steps in trajs.items():
        step_map = {t: (x, y) for t, x, y, _ in steps}
        row = {}
        last_xy = None
        for t in range(t_max + 1):
            if t in step_map:
                last_xy = step_map[t]
            row[t] = last_xy   # None if trajectory hasn't started yet
        pos[aid] = row
    return pos, t_max


def check_vertex_conflicts(trajs):
    """
    Report any (t, x, y) occupied by 2+ agents, including agents that have
    finished their trajectory and are staying at their final position.
    Returns list of (t, x, y, agent_i, agent_j) tuples.
    """
    pos, t_max = _build_pos_table(trajs)
    agents = sorted(trajs.keys())

    conflicts = []
    for t in range(t_max + 1):
        cell_at = defaultdict(list)
        for aid in agents:
            xy = pos[aid][t]
            if xy is not None:
                cell_at[xy].append(aid)
        for xy, occupants in cell_at.items():
            if len(occupants) >= 2:
                for i in range(len(occupants)):
                    for j in range(i + 1, len(occupants)):
                        conflicts.append((t, xy[0], xy[1],
                                          occupants[i], occupants[j]))
    return conflicts


def check_edge_conflicts(trajs):
    """
    Detect swap conflicts: agents i and j swap cells between t and t+1.
    (Cases where both agents are already at the same cell are vertex conflicts
    and are excluded here to avoid double-reporting.)
    Returns list of (t, agent_i, agent_j, x1, y1, x2, y2).
    """
    pos, t_max = _build_pos_table(trajs)
    agents = sorted(trajs.keys())

    conflicts = []
    for t in range(t_max):
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                ai, aj = agents[i], agents[j]
                p1i = pos[ai][t];     p1j = pos[aj][t]
                p2i = pos[ai][t+1];   p2j = pos[aj][t+1]
                if None in (p1i, p1j, p2i, p2j):
                    continue
                if p1i == p1j:
                    continue   # already a vertex conflict; skip
                if p1i == p2j and p1j == p2i:
                    conflicts.append((t, ai, aj, p1i[0], p1i[1], p1j[0], p1j[1]))
    return conflicts


def check_wall_collisions(trajs, grid, thresh):
    """
    Report any step where an agent occupies a cell with value >= thresh.
    Returns list of (t, agent, x, y, cell_value).
    """
    violations = []
    for aid, steps in trajs.items():
        for t, x, y, _ in steps:
            v = grid.get((x, y), thresh)
            if v >= thresh:
                violations.append((t, aid, x, y, v))
    return violations


def check_move_validity(trajs):
    """
    Check that consecutive steps in each agent's trajectory correspond to
    a legal action: forward, backward, 2-step turn-left, 2-step turn-right,
    or wait. A turn must be completed from the forward cell reached in the
    immediately preceding timestep.
    Returns list of (agent, t, reason) for illegal transitions.
    """
    violations = []
    for aid, steps in trajs.items():
        for k in range(len(steps) - 1):
            t0, x0, y0, h0 = steps[k]
            t1, x1, y1, h1 = steps[k + 1]

            if t1 != t0 + 1:
                continue   # non-consecutive timesteps — skip gap check

            if h0 < 0 or h1 < 0:
                continue   # heading unknown, can't validate move

            dx, dy = x1 - x0, y1 - y0
            dh = (h1 - h0) % 4
            left_h = (h0 + 1) % 4
            right_h = (h0 + 3) % 4

            # Legal transitions:
            #   wait        : dx=dy=0, dh=0
            #   forward     : (dx,dy)=(DX[h0],DY[h0]), dh=0
            #   backward    : (dx,dy)=(-DX[h0],-DY[h0]), dh=0
            #   turn-left   : prior step moved forward with heading h0,
            #                 then (dx,dy)=(DX[left_h],DY[left_h]), dh=1
            #   turn-right  : prior step moved forward with heading h0,
            #                 then (dx,dy)=(DX[right_h],DY[right_h]), dh=3
            if dx == 0 and dy == 0:
                if dh != 0:
                    violations.append((aid, t0,
                        'in-place turns are no longer allowed'))
            elif dh == 0:
                fwd = (DX[h0], DY[h0])
                bwd = (-DX[h0], -DY[h0])
                if (dx, dy) not in (fwd, bwd):
                    violations.append((aid, t0,
                        f'illegal move direction: '
                        f'({x0},{y0},h={h0})->({x1},{y1})'))
            elif dh == 1:
                ok = (dx, dy) == (DX[left_h], DY[left_h])
                if ok and k > 0:
                    tp, xp, yp, hp = steps[k - 1]
                    ok = (tp == t0 - 1 and hp == h0 and
                          (x0 - xp, y0 - yp) == (DX[h0], DY[h0]))
                else:
                    ok = False
                if not ok:
                    violations.append((aid, t0,
                        f'illegal left turn: '
                        f'({x0},{y0},h={h0})->({x1},{y1},h={h1})'))
            elif dh == 3:
                ok = (dx, dy) == (DX[right_h], DY[right_h])
                if ok and k > 0:
                    tp, xp, yp, hp = steps[k - 1]
                    ok = (tp == t0 - 1 and hp == h0 and
                          (x0 - xp, y0 - yp) == (DX[h0], DY[h0]))
                else:
                    ok = False
                if not ok:
                    violations.append((aid, t0,
                        f'illegal right turn: '
                        f'({x0},{y0},h={h0})->({x1},{y1},h={h1})'))
            else:
                violations.append((aid, t0,
                    f'illegal heading change: heading {h0}->{h1}'))
    return violations


def _infer_wall_facing_heading(map_data, x, y):
    grid = map_data['grid']
    thresh = map_data.get('thresh', 100)
    if grid.get((x, y), thresh) != 1:
        return None

    def is_wall(nx, ny):
        if nx < 0 or nx >= map_data['W'] or ny < 0 or ny >= map_data['H']:
            return True
        return grid.get((nx, ny), thresh) >= thresh

    north_wall = is_wall(x, y + 1)
    south_wall = is_wall(x, y - 1)
    if north_wall != south_wall:
        return 1 if north_wall else 3

    found = None
    for h in range(4):
        if not is_wall(x + DX[h], y + DY[h]):
            continue
        if found is not None:
            return None
        found = h
    return found


def check_goal_conditions(trajs, map_data):
    """
    Check that each recorded trajectory ends at its assigned goal cell and,
    when a goal heading is known or inferable, ends with that heading.
    Returns list of (agent, reason).
    """
    goals = map_data.get('agent_goals', [])
    violations = []

    for aid, steps in trajs.items():
        if not steps or aid >= len(goals):
            continue

        gx, gy, gh = goals[aid]
        final_t, fx, fy, fh = steps[-1]
        if (fx, fy) != (gx, gy):
            violations.append((aid,
                f'final state at t={final_t} ends at ({fx},{fy}) instead of goal ({gx},{gy})'))
            continue

        required_h = gh if gh is not None else _infer_wall_facing_heading(map_data, gx, gy)
        if required_h is not None and fh >= 0 and fh != required_h:
            violations.append((aid,
                f'final heading {fh} does not face separator wall at goal ({gx},{gy}); expected {required_h}'))
    return violations


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Check trajectory validity.')
    ap.add_argument('map',  help='Map file (N/C/A/M format)')
    ap.add_argument('traj', help='Trajectory file (agent_id,t,x,y[,heading])')
    ap.add_argument('--verbose', '-v', action='store_true',
                    help='Print every violation (default: summary only)')
    args = ap.parse_args()

    print(f'Map:  {args.map}')
    print(f'Traj: {args.traj}')
    print()

    map_data = parse_map(args.map)
    trajs    = parse_trajectories(args.traj)

    W      = map_data['W']
    H      = map_data['H']
    thresh = map_data.get('thresh', 100)
    grid   = map_data['grid']

    n_agents = len(trajs)
    t_max    = max(t for steps in trajs.values() for t, *_ in steps)
    n_steps  = sum(len(s) for s in trajs.values())
    print(f'Agents: {n_agents}  |  Makespan: {t_max}  |  Total steps: {n_steps}')
    print()

    all_ok = True

    # 1. Vertex conflicts
    vc = check_vertex_conflicts(trajs)
    if vc:
        all_ok = False
        print(f'[FAIL] Vertex conflicts: {len(vc)}')
        if args.verbose:
            for t, x, y, ai, aj in vc[:50]:
                print(f'       t={t}  cell=({x},{y})  agents {ai} and {aj}')
            if len(vc) > 50:
                print(f'       … and {len(vc)-50} more')
    else:
        print('[PASS] No vertex conflicts')

    # 2. Edge (swap) conflicts
    ec = check_edge_conflicts(trajs)
    if ec:
        all_ok = False
        print(f'[FAIL] Edge (swap) conflicts: {len(ec)}')
        if args.verbose:
            for t, ai, aj, x1, y1, x2, y2 in ec[:20]:
                print(f'       t={t}->{t+1}  agents {ai} and {aj} '
                      f'swap ({x1},{y1})<->({x2},{y2})')
    else:
        print('[PASS] No edge (swap) conflicts')

    # 3. Wall collisions
    wc = check_wall_collisions(trajs, grid, thresh)
    if wc:
        all_ok = False
        print(f'[FAIL] Wall/obstacle collisions: {len(wc)}')
        if args.verbose:
            for t, aid, x, y, v in wc[:20]:
                print(f'       agent {aid}  t={t}  cell=({x},{y})  value={v}')
    else:
        print('[PASS] No wall collisions')

    # 4. Move validity
    mv = check_move_validity(trajs)
    if mv:
        all_ok = False
        print(f'[FAIL] Illegal moves: {len(mv)}')
        if args.verbose:
            for aid, t, reason in mv[:20]:
                print(f'       agent {aid}  t={t}  {reason}')
    else:
        print('[PASS] All moves are legal')

    # 5. Goal completion / parked heading
    gc = check_goal_conditions(trajs, map_data)
    if gc:
        all_ok = False
        print(f'[FAIL] Goal pose violations: {len(gc)}')
        if args.verbose:
            for aid, reason in gc[:20]:
                print(f'       agent {aid}  {reason}')
            if len(gc) > 20:
                print(f'       … and {len(gc)-20} more')
    else:
        print('[PASS] All recorded trajectories end at valid goal poses')

    print()
    if all_ok:
        print('RESULT: OK — trajectories are valid.')
        sys.exit(0)
    else:
        print('RESULT: INVALID — see failures above.')
        sys.exit(1)


if __name__ == '__main__':
    main()
