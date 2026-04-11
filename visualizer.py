#!/usr/bin/env python3
"""
Parking lot visualizer for multi-agent autonomous vehicle planning.

Algorithm-agnostic: reads a map file and an optional trajectory file
produced by any planner.

─── Usage ────────────────────────────────────────────────────────────────────
  # Static map (PNG)
  python visualizer.py map/parking_lot.txt --save output/map.png

  # Animate planner output (GIF)
  python visualizer.py map/parking_lot.txt \\
      --traj output/trajectories.txt --animate --save output/result.gif

─── Map file format ──────────────────────────────────────────────────────────
  N
  80,64                ← x_size, y_size
  C
  100                  ← collision threshold
  A
  4                    ← number of agents
  2,63,N               ← agent start: x, y, heading  (N/S/E/W or 0-3)
  6,6                  ← agent goal:  x, y  (heading optional)
  …
  M
  100,100,…            ← costmap row y=0  (x=0 .. x_size-1)
  …

  Cell values: 0=road  1=parking spot  >=100=wall
  Headings:    0=East  1=North  2=West  3=South

─── Trajectory file format ───────────────────────────────────────────────────
  # Lines starting with # are comments
  agent_id,timestep,x,y,heading
  0,0,2,63,1
  0,1,2,59,1
  …

  heading column is optional (omit if your planner doesn't track it).
  One line per (agent, timestep).  Fractional timesteps are OK.
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.markers import MarkerStyle

# ── Colour scheme ──────────────────────────────────────────────────────────
WALL_COLOR  = '#2e2e2e'   # dark charcoal
ROAD_COLOR  = '#c8c8c8'   # light gray (asphalt)
SPOT_COLOR  = '#a8c8e8'   # steel blue (empty parking space)

# Fixed palette for small N (≤10); colormap used for larger N
_AGENT_PALETTE = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12',
    '#9b59b6', '#1abc9c', '#e67e22', '#e91e63',
    '#00bcd4', '#8bc34a',
]


def _agent_color(i: int, total: int):
    """Return an RGB/hex color for agent i out of total agents."""
    if total <= len(_AGENT_PALETTE):
        return _AGENT_PALETTE[i]
    # For many agents use a perceptually-uniform cyclic colormap
    return matplotlib.colormaps['hsv'](i / total)

# heading → (dx, dy) unit vector  (0=E 1=N 2=W 3=S)
HEADING_DXY = {0: (1, 0), 1: (0, 1), 2: (-1, 0), 3: (0, -1)}
# heading → rotation of '^' marker in degrees  ('^' points North by default)
HEADING_ROT  = {0: -90, 1: 0, 2: 90, 3: 180}
HEADING_NAMES = {0: 'E', 1: 'N', 2: 'W', 3: 'S'}


# ── Heading helpers ────────────────────────────────────────────────────────

def _parse_heading(s) -> int | None:
    """Accept 'N'/'S'/'E'/'W' or '0'-'3'; return int 0-3 or None."""
    if s is None:
        return None
    s = str(s).strip().upper()
    mapping = {'E': 0, 'N': 1, 'W': 2, 'S': 3,
               '0': 0, '1': 1, '2': 2, '3': 3}
    return mapping.get(s)


def _car_marker(heading: int) -> MarkerStyle:
    """Return a '^' MarkerStyle rotated to point in heading direction."""
    m = MarkerStyle('^')
    m._transform = m.get_transform().rotate_deg(HEADING_ROT.get(heading, 0))
    return m


# ── Map parsing ────────────────────────────────────────────────────────────

def parse_map(filepath: str) -> dict:
    with open(filepath) as f:
        lines = [l.rstrip('\n') for l in f if l.strip()]

    result = {
        'x_size': 0, 'y_size': 0, 'collision_thresh': 100,
        'num_agents': 0,
        'agent_starts': [],   # list of (x, y, heading_int_or_None)
        'agent_goals':  [],   # list of (x, y, heading_int_or_None)
        'costmap': None,
    }
    i = 0
    while i < len(lines):
        hdr = lines[i].strip(); i += 1
        if hdr == 'N':
            result['x_size'], result['y_size'] = map(int, lines[i].split(',')); i += 1
        elif hdr == 'C':
            result['collision_thresh'] = int(lines[i]); i += 1
        elif hdr == 'A':
            n = int(lines[i]); i += 1
            result['num_agents'] = n
            for _ in range(n):
                sp = lines[i].split(','); i += 1
                sx, sy = int(sp[0]), int(sp[1])
                sh = _parse_heading(sp[2]) if len(sp) > 2 else None
                result['agent_starts'].append((sx, sy, sh))

                gp = lines[i].split(','); i += 1
                gx, gy = int(gp[0]), int(gp[1])
                gh = _parse_heading(gp[2]) if len(gp) > 2 else None
                result['agent_goals'].append((gx, gy, gh))
        elif hdr == 'M':
            W, H = result['x_size'], result['y_size']
            costmap = np.zeros((W, H), dtype=int)
            for y in range(H):
                vals = list(map(int, lines[i].split(','))); i += 1
                costmap[:, y] = vals
            result['costmap'] = costmap
    return result


def parse_trajectories(filepath: str, num_agents: int) -> list:
    """
    Parse trajectory file: agent_id,time,x,y[,heading] per line.
    Returns list of np.array shape (T, 4): [time, x, y, heading(-1=unknown)].
    """
    raw = [[] for _ in range(num_agents)]
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split(',')
            if len(p) < 4:
                continue
            aid = int(p[0])
            t, x, y = float(p[1]), float(p[2]), float(p[3])
            h = _parse_heading(p[4]) if len(p) > 4 else None
            if 0 <= aid < num_agents:
                raw[aid].append([t, x, y, h if h is not None else -1])
    return [np.array(r) if r else np.empty((0, 4)) for r in raw]


# ── Map image ──────────────────────────────────────────────────────────────

def _build_rgb(costmap: np.ndarray, collision_thresh: int) -> np.ndarray:
    """Return float (H, W, 3) RGB array for imshow with origin='lower'."""
    import matplotlib.colors as mc
    W, H = costmap.shape
    wall_rgb = np.array(mc.to_rgb(WALL_COLOR))
    road_rgb = np.array(mc.to_rgb(ROAD_COLOR))
    spot_rgb = np.array(mc.to_rgb(SPOT_COLOR))
    img = np.full((H, W, 3), wall_rgb)
    for x in range(W):
        for y in range(H):
            v = costmap[x, y]
            if v >= collision_thresh:
                img[y, x] = wall_rgb
            elif v == 1:
                img[y, x] = spot_rgb
            else:
                img[y, x] = road_rgb
    return img


def _add_stall_lines(ax, costmap: np.ndarray):
    """Draw thin white lines at parking-spot cell boundaries."""
    W, H = costmap.shape
    is_spot = (costmap == 1)
    segs = []
    xi, yi = np.where(is_spot[:-1, :] != is_spot[1:, :])
    for x, y in zip(xi, yi):
        segs.append([(x + 0.5, y - 0.5), (x + 0.5, y + 0.5)])
    xi, yi = np.where(is_spot[:, :-1] != is_spot[:, 1:])
    for x, y in zip(xi, yi):
        segs.append([(x - 0.5, y + 0.5), (x + 0.5, y + 0.5)])
    if segs:
        ax.add_collection(LineCollection(segs, colors='white',
                                         linewidths=0.5, alpha=0.55, zorder=3))


def _setup_axes(ax, map_data: dict, title: str):
    img = _build_rgb(map_data['costmap'], map_data['collision_thresh'])
    ax.imshow(img, origin='lower', interpolation='nearest')
    _add_stall_lines(ax, map_data['costmap'])
    W, H = map_data['x_size'], map_data['y_size']
    ax.set_xlim(-0.5, W - 0.5)
    ax.set_ylim(-0.5, H - 0.5)
    ax.set_title(title, fontsize=13, pad=8)
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.tick_params(labelsize=8)


# ── Agent markers ──────────────────────────────────────────────────────────

def _draw_starts_goals(ax, map_data: dict, alpha: float = 0.9):
    """
    Start: filled triangle pointing in the agent's initial heading.
    Goal:  hollow star.
    """
    for i, (s, g) in enumerate(zip(map_data['agent_starts'],
                                   map_data['agent_goals'])):
        sx, sy, sh = s
        gx, gy, _  = g
        n = map_data['num_agents']
        c = _agent_color(i, n)
        label = n <= 20  # only annotate S/G labels when there are few agents

        # Start marker: rotated triangle pointing in initial heading
        marker = _car_marker(sh) if sh is not None else 'o'
        ms = 11 if n <= 20 else 6
        ax.plot(sx, sy, marker=marker, color=c, markersize=ms,
                markeredgecolor='white', markeredgewidth=0.8,
                zorder=6, alpha=alpha, linestyle='None')

        # Goal marker: star (scaled down for many agents)
        gs = 13 if n <= 20 else 5
        ax.plot(gx, gy, '*', color=c, markersize=gs,
                markeredgecolor='white', markeredgewidth=0.5,
                zorder=6, alpha=alpha)

        if label:
            ax.annotate(f'S{i}', xy=(sx, sy), xytext=(sx+0.9, sy+0.9),
                        fontsize=6, color='white', fontweight='bold', zorder=7)
            ax.annotate(f'G{i}', xy=(gx, gy), xytext=(gx+0.9, gy+0.9),
                        fontsize=6, color='white', fontweight='bold', zorder=7)


def _make_legend(map_data: dict, with_agents: bool = True) -> list:
    elems = [
        mpatches.Patch(facecolor=WALL_COLOR, label='Wall / divider'),
        mpatches.Patch(facecolor=ROAD_COLOR, label='Road / lane'),
        mpatches.Patch(facecolor=SPOT_COLOR, label='Parking spot'),
    ]
    n = map_data['num_agents']
    if with_agents and n:
        if n <= 10:
            # Show one legend entry per agent
            for i in range(n):
                elems.append(Line2D([0], [0], marker='^', color='w',
                                    markerfacecolor=_agent_color(i, n),
                                    markersize=9, label=f'Agent {i}'))
        else:
            # Too many to list individually — just indicate the count
            elems.append(Line2D([0], [0], marker='^', color='w',
                                markerfacecolor='grey', markersize=9,
                                label=f'{n} agents (color-coded)'))
    return elems


# ── Static view ────────────────────────────────────────────────────────────

def static_view(map_data: dict, show_agents: bool = True,
                save_path: str = None):
    fig, ax = plt.subplots(figsize=(14, 11))
    _setup_axes(ax, map_data, 'Parking Lot — Multi-Agent Planning Map')
    if show_agents and map_data['num_agents']:
        _draw_starts_goals(ax, map_data)
    ax.legend(handles=_make_legend(map_data, with_agents=show_agents),
              loc='upper right', fontsize=8, framealpha=0.9)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {save_path}')
    else:
        plt.show()
    plt.close()


# ── Animation ──────────────────────────────────────────────────────────────

def _interp(traj: np.ndarray, t: float):
    """Return (x, y, heading_or_None) at time t via linear interpolation."""
    if len(traj) == 0:
        return None
    ts = traj[:, 0]
    if t <= ts[0]:
        r = traj[0]
    elif t >= ts[-1]:
        r = traj[-1]
    else:
        idx = int(np.searchsorted(ts, t)) - 1
        t0, t1 = ts[idx], ts[idx + 1]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        r = traj[idx] + a * (traj[idx + 1] - traj[idx])
    x, y = r[1], r[2]
    h = int(round(r[3])) if len(r) > 3 and r[3] >= 0 else None
    return x, y, h


def animate_view(map_data: dict, trajectories: list,
                 save_path: str = 'output/result.gif', fps: int = 5):
    n = map_data['num_agents']
    non_empty = [t for t in trajectories if len(t) > 0]
    t_max = int(max(t[:, 0].max() for t in non_empty)) if non_empty else 0

    fig, ax = plt.subplots(figsize=(14, 11))
    _setup_axes(ax, map_data, 'Parking Lot — Multi-Agent Planning Animation')
    _draw_starts_goals(ax, map_data, alpha=0.45)

    # Per-agent artists:
    #   path_line  – trail of visited cells
    #   car_dot    – circle body  (scatter, easy to update)
    #   dir_arrow  – line showing heading direction
    path_lines, car_dots, dir_arrows = [], [], []
    dot_size  = 12 if n <= 20 else 6   # shrink dots for many agents
    for i in range(n):
        c = _agent_color(i, n)
        line, = ax.plot([], [], '-', color=c, alpha=0.35, lw=1.5, zorder=4)
        dot,  = ax.plot([], [], 'o', color=c, markersize=dot_size,
                        markeredgecolor='white', markeredgewidth=1.0, zorder=8)
        arr,  = ax.plot([], [], '-', color='white', lw=2.0, zorder=9)
        path_lines.append(line)
        car_dots.append(dot)
        dir_arrows.append(arr)

    time_text = ax.text(
        0.02, 0.97, '', transform=ax.transAxes, fontsize=11,
        va='top', bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.85),
        zorder=10,
    )
    ax.legend(handles=_make_legend(map_data), loc='upper right',
              fontsize=8, framealpha=0.9)

    def update(frame):
        time_text.set_text(f't = {frame}')
        artists = [time_text]
        for i, (traj, pline, dot, arr) in enumerate(
                zip(trajectories, path_lines, car_dots, dir_arrows)):
            if len(traj) == 0:
                continue
            past = traj[traj[:, 0] <= frame]
            if len(past):
                pline.set_data(past[:, 1], past[:, 2])
            result = _interp(traj, frame)
            if result:
                x, y, h = result
                dot.set_data([x], [y])
                if h is not None:
                    dx, dy = HEADING_DXY[h]
                    # Draw a short line from center in the heading direction
                    arr.set_data([x - dx * 0.3, x + dx * 0.55],
                                 [y - dy * 0.3, y + dy * 0.55])
                else:
                    arr.set_data([], [])
            artists += [pline, dot, arr]
        return artists

    ani = animation.FuncAnimation(
        fig, update, frames=range(t_max + 1),
        interval=int(1000 / fps), blit=True,
    )

    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
    ext = save_path.rsplit('.', 1)[-1].lower()
    writer = 'pillow' if ext == 'gif' else 'ffmpeg'
    ani.save(save_path, writer=writer, fps=fps)
    print(f'Saved animation: {save_path}')
    plt.close()


# ── CLI ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Parking lot visualizer for multi-agent planning.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('map', help='Map file path')
    parser.add_argument('--traj', metavar='FILE',
                        help='Trajectory file (agent_id,time,x,y[,heading])')
    parser.add_argument('--animate', action='store_true',
                        help='Animate trajectories (requires --traj)')
    parser.add_argument('--save', metavar='FILE',
                        help='Save output (.png static | .gif/.mp4 animation)')
    parser.add_argument('--fps', type=int, default=5,
                        help='Animation FPS (default 5)')
    parser.add_argument('--no-agents', action='store_true',
                        help='Hide agent markers on static view')
    args = parser.parse_args()

    map_data = parse_map(args.map)
    print(f"Map: {map_data['x_size']}×{map_data['y_size']}  |  "
          f"agents: {map_data['num_agents']}  |  "
          f"collision_thresh: {map_data['collision_thresh']}")

    if args.animate:
        if not args.traj:
            print('Error: --animate requires --traj', file=sys.stderr)
            sys.exit(1)
        trajs = parse_trajectories(args.traj, map_data['num_agents'])
        animate_view(map_data, trajs,
                     save_path=args.save or 'output/result.gif',
                     fps=args.fps)
    else:
        static_view(map_data,
                    show_agents=not args.no_agents,
                    save_path=args.save)


if __name__ == '__main__':
    main()
