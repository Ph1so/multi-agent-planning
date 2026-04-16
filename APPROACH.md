# Multi-Agent Parking Lot Planner — Design & Implementation Summary

## Overview

This project implements **Multi-Agent Path Finding (MAPF)** for autonomous vehicles navigating a shared parking lot. Multiple agents must move from perimeter entry points to assigned parking spots without colliding. The system includes a C++ planning engine, Python map generators, a trajectory validator, and an algorithm-agnostic visualizer.

---

## Algorithms

### Conflict-Based Search (CBS) — Optimal

CBS is a two-level algorithm that finds provably optimal, collision-free paths.

**High-level (Constraint Tree):**
- Maintains a min-heap of constraint-tree (CT) nodes, ordered by total path cost (sum of all agent path lengths)
- Each CT node holds a set of constraints and one path per agent
- At each expansion, detects the first conflict in the current joint solution
- Branches into two children by adding one new constraint — one for each conflicting agent
- Terminates when a CT node has no conflicts; that solution is optimal

**Low-level (Space-Time A\*):**
- Plans a single agent through 4D state space: `(x, y, heading, t)`
- Respects vertex constraints `(agent, x, y, t)` and edge constraints `(agent, x1, y1, x2, y2, t)`
- Heuristic: precomputed BFS distance from goal, ignoring heading and time — admissible because heading changes cost ≤ 1 extra step per move
- 64-bit state encoding for O(1) closed-set lookup: `(t << 20) | (x << 10) | (y << 4) | heading`

**Practical limits:** CBS is exponential in the number of conflicts. A hard cap of `MAX_CBS_NODES = 10,000` prevents runaway search; the best solution found so far is returned if exceeded. Reliable up to ~4–6 agents on the main lot.

---

### Prioritized Planning (PP) — Fast, Suboptimal

PP plans agents one at a time in a fixed priority order, treating previously planned agents as dynamic obstacles.

**Priority ordering:** Agents with a lower `start.y` (closer to lot interior) are planned first. This prevents higher-priority interior agents from being blocked by lower-priority perimeter agents.

**Pass 1:** Each agent runs space-time A\* against a growing `ReservedSet` — a set of `(x, y, t)` tuples from all previously planned paths. Swap conflicts (two agents trading cells) are also blocked.

**Parking completion check:** Before accepting a goal cell as reached, the planner verifies that no future reservation exists at `(gx, gy, t > cur.t)`. This prevents an agent from "parking" in a cell that a later agent will drive through.

**Pass 2 (recovery):** If any agents fail in Pass 1, their start cells are permanently blocked and all other agents are replanned. This breaks the deadlock where a failed agent's lingering start position blocks others.

**Practical limits:** Reliable up to ~16 agents on the main lot; not optimal but fast.

---

## Data Structures

### Map (`Map`)
```cpp
struct Map {
    int W, H, collision_thresh;
    std::vector<std::vector<int>> grid;  // grid[x][y]: 0=road, 1=parking, ≥100=wall
    std::vector<AgentDef> agents;
};
```

### Agent (`AgentDef`)
```cpp
struct AgentDef {
    int sx, sy, sh;  // start position and heading (0=E, 1=N, 2=W, 3=S)
    int gx, gy;      // goal position
};
```

### Path (`Path`)
```cpp
struct Step { int t, x, y, h; };
using Path = std::vector<Step>;
```

### Constraints (CBS)
```cpp
struct VertexConstraint { int agent, x, y, t; };
struct EdgeConstraint   { int agent, x1, y1, x2, y2, t; };
```

Constraints are stored per CT node and checked inline during low-level A\* expansion.

### Reservations (PP)
```cpp
using ReservedSet = std::set<std::tuple<int,int,int>>;  // (x, y, t)
```
Using `std::set` enables `upper_bound` range queries to efficiently check whether any future reservation exists at a goal cell.

### A\* Node
```cpp
struct AStarNode {
    int f, g, x, y, h, t;
    bool operator>(const AStarNode& o) const { return f > o.f; }
};
```
Stored in a `std::priority_queue` (min-heap by f-value).

---

## Motion Model

Each agent has **5 actions**, all with uniform cost 1:

| Action | Effect |
|--------|--------|
| FORWARD | `(x,y) → (x + DX[h], y + DY[h])` |
| BACKWARD | `(x,y) → (x − DX[h], y − DY[h])` |
| TURN_LEFT | `heading = (h + 1) % 4` |
| TURN_RIGHT | `heading = (h + 3) % 4` |
| WAIT | no change |

Heading deltas: `DX = {1, 0, -1, 0}`, `DY = {0, 1, 0, -1}` for East/North/West/South.

An agent **cannot move and turn in the same timestep** — motion and rotation are independent actions.

---

## Heuristic

For each agent, the low-level A\* uses a precomputed **BFS distance map** from the goal cell:

```cpp
std::vector<std::vector<int>> bfs_heuristic(const Map& map, int gx, int gy);
```

- BFS runs on the 2D grid ignoring heading and time constraints
- Returns `dist[x][y]` = minimum number of moves from `(x,y)` to `(gx, gy)`
- **Admissible:** the BFS distance lower-bounds true cost since heading changes cost at most 1 turn per cell traversed
- **Efficient:** precomputed once per agent per CT node (low-level replanning still uses the same BFS map)

---

## Conflict Detection

Two conflict types are detected when comparing two agent paths step-by-step:

**Vertex conflict:** Both agents occupy the same cell at the same time.
```
agent_i at (x,y,t) == agent_j at (x,y,t)
```

**Edge (swap) conflict:** Two agents trade cells between consecutive timesteps.
```
agent_i: (x1,y1,t) → (x2,y2,t+1)
agent_j: (x2,y2,t) → (x1,y1,t+1)
```

CBS resolves each conflict by adding a constraint to one of the two agents (branching into two CT children).

---

## Map File Format

Maps are plain text with four labeled sections:

```
N
<width>,<height>
C
<collision_threshold>        # cells with value ≥ this are walls
A
<num_agents>
<sx>,<sy>,<heading>          # start for agent i (heading: N/S/E/W or 0–3)
<gx>,<gy>                    # goal for agent i
...
M
<row of comma-separated cell values for y=0>
<row for y=1>
...
```

**Cell values:** `0` = road, `1` = parking spot, `≥100` = wall/obstacle.

### Map Scenarios

| Map | Dimensions | Max Agents | Challenge |
|-----|-----------|------------|-----------|
| Main parking lot | 28×64 | 144 | Dense interior navigation |
| Gauntlet | 16×24 | 24 | Bidirectional 2-cell bottleneck |
| Two Lots | 48×38 | 16 | Cross-lot travel via 3 roads |
| Two Lots Gauntlet | 30×26 | — | Compact bottleneck variant |

---

## Trajectory Output Format

Trajectories are written as CSV (one row per agent-step):

```
# agent_id,timestep,x,y,heading  (0=E 1=N 2=W 3=S)
0,0,2,63,1
0,1,2,62,1
1,0,3,63,1
...
```

Each agent's rows are sorted by timestep. Agents that reach their goal early are extended with a wait step at the goal for the remainder of the simulation.

---

## Visualizer

The visualizer (`visualizer.py`) is **algorithm-agnostic** — it reads map and trajectory files and renders them independently of how they were produced.

### Static View

Produces a PNG of the map with agent starts and goals:

1. Builds an RGB image from the costmap:
   - Wall → dark charcoal (`#2e2e2e`)
   - Road → light gray (`#c8c8c8`)
   - Parking spot → steel blue (`#a8c8e8`)
2. Overlays parking-stall boundaries using `LineCollection`
3. Draws start markers (filled triangle, rotated to initial heading) and goal markers (hollow star)
4. Adds a legend and saves at 150 dpi

### Animation

Produces a GIF (or MP4) animating agent trajectories:

1. Uses `matplotlib.animation.FuncAnimation` for frame-by-frame updates
2. Per frame:
   - **Path trail:** semi-transparent polyline of agent history
   - **Car dot:** colored circle at current position
   - **Direction arrow:** short white line indicating heading
   - **Timestep counter:** displayed in corner
3. Supports fractional-timestep interpolation for smooth playback
4. Color scheme: fixed 10-color palette for N ≤ 10 agents; HSV colormap for larger N

### Libraries

- `matplotlib` — plotting, animation, markers
- `numpy` — costmap array operations
- `matplotlib.animation` — `FuncAnimation`, pillow/ffmpeg writers

---

## Checker / Validator

`checker.py` validates a trajectory file against a map, checking:

- **Vertex conflicts:** two agents at same `(x, y, t)`
- **Edge conflicts:** two agents swapping cells between timesteps
- **Wall collisions:** agent moves into a cell with value ≥ `collision_thresh`
- **Illegal moves:** any step that doesn't match one of the 5 valid actions (forward, backward, turn-left, turn-right, wait)

The checker is independent of the planner and can validate trajectories from any source.

---

## Design Choices & Tradeoffs

| Decision | Rationale |
|----------|-----------|
| CBS + PP dual-algorithm | CBS gives optimal solutions for small N; PP handles larger N where CBS times out |
| BFS heuristic (not Euclidean) | More accurate for grid-constrained motion; admissible without heading overhead |
| 64-bit state key | Avoids tuple-hashing overhead in the closed set; critical for A\* performance |
| `std::set` for reservations (PP) | Enables `upper_bound` to check "any future reservation at goal" in O(log n) |
| Heading as part of state | Turns cost time, so heading affects reachability — omitting it would make the heuristic inadmissible |
| Two-pass recovery (PP) | Single-pass PP can deadlock when a stuck agent blocks others; Pass 2 breaks the cycle without full replanning |
| Uniform action cost | Simplifies A\* and CBS cost accounting; turning is just as "expensive" as moving, which is realistic for AV maneuvering |
| Parking completion check | Prevents agents from falsely "completing" in cells that future agents will traverse |



Notes:

motion prims have different time steps
