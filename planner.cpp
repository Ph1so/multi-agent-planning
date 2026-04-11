/**
 * planner.cpp — Conflict-Based Search (CBS) for multi-agent parking lot
 *
 * State:  (x, y, heading)   heading: 0=E 1=N 2=W 3=S
 * Actions per timestep (all cost 1):
 *   FORWARD   — move +1 cell in heading direction
 *   BACKWARD  — move -1 cell in heading direction
 *   TURN_LEFT  — heading = (heading + 1) % 4  (no position change)
 *   TURN_RIGHT — heading = (heading + 3) % 4
 *   WAIT      — stay
 *
 * Reads:  map/parking_lot.txt  (N/C/A/M format)
 * Writes: output/trajectories.txt  (agent_id,timestep,x,y,heading)
 *
 * Build:
 *   g++ -O2 -std=c++17 -o planner planner.cpp
 * Run (examples):
 *   ./planner                                         # all agents, CBS
 *   ./planner map/parking_lot.txt output/traj.txt     # explicit paths
 *   ./planner --max-agents 4                          # limit to first 4 agents
 *   ./planner --pp                                    # prioritized planning (fast)
 */

#include <algorithm>
#include <cassert>
#include <numeric>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// ── Constants ────────────────────────────────────────────────────────────────

static const int COLLISION_DEFAULT = 100;
static const int MAX_TIMESTEP      = 800;   // hard cap to avoid runaway search
static const int MAX_CBS_NODES     = 10000; // bail out after this many CT nodes

// heading deltas: East(0) North(1) West(2) South(3)
static const int DX[4] = { 1,  0, -1,  0};
static const int DY[4] = { 0,  1,  0, -1};

// ── Data types ───────────────────────────────────────────────────────────────

struct AgentDef {
    int sx, sy, sh;   // start (heading 0-3)
    int gx, gy;       // goal  (any heading at goal)
};

// A single position in a trajectory
struct Step {
    int t, x, y, h;
};
using Path = std::vector<Step>;   // time-ordered path for one agent

// ── Constraint types ─────────────────────────────────────────────────────────

struct VertexConstraint { int agent, x, y, t; };
struct EdgeConstraint   { int agent, x1, y1, x2, y2, t; };  // agent moves (x1,y1)->(x2,y2) at time t

struct Constraints {
    std::vector<VertexConstraint> vert;
    std::vector<EdgeConstraint>   edge;
};

// ── Map ──────────────────────────────────────────────────────────────────────

struct Map {
    int W, H, collision_thresh;
    std::vector<std::vector<int>> grid;   // grid[x][y]
    std::vector<AgentDef> agents;

    bool passable(int x, int y) const {
        if (x < 0 || x >= W || y < 0 || y >= H) return false;
        return grid[x][y] < collision_thresh;
    }
};

// ── Heading helpers ───────────────────────────────────────────────────────────

static int parse_heading(const std::string& s) {
    if (s == "E" || s == "0") return 0;
    if (s == "N" || s == "1") return 1;
    if (s == "W" || s == "2") return 2;
    if (s == "S" || s == "3") return 3;
    return 1;  // default North
}

// ── Map file parser ───────────────────────────────────────────────────────────

static Map load_map(const std::string& path, int max_agents = INT_MAX) {
    std::ifstream f(path);
    if (!f) { std::cerr << "Cannot open map: " << path << "\n"; exit(1); }

    Map m;
    m.collision_thresh = COLLISION_DEFAULT;
    std::string line, tok;

    auto read_tag = [&](char expected) {
        while (std::getline(f, line))
            if (!line.empty() && line[0] == expected) return;
        std::cerr << "Expected tag '" << expected << "'\n"; exit(1);
    };

    read_tag('N');
    std::getline(f, line);
    { std::istringstream ss(line);
      std::getline(ss, tok, ','); m.W = std::stoi(tok);
      std::getline(ss, tok, ','); m.H = std::stoi(tok); }

    read_tag('C');
    std::getline(f, line);
    m.collision_thresh = std::stoi(line);

    read_tag('A');
    std::getline(f, line);
    int n_agents = std::stoi(line);
    int load_n   = std::min(n_agents, max_agents);

    for (int i = 0; i < n_agents; ++i) {
        std::string sline, gline;
        std::getline(f, sline);
        std::getline(f, gline);
        if (i >= load_n) continue;
        AgentDef a;
        { std::istringstream ss(sline);
          std::getline(ss, tok, ','); a.sx = std::stoi(tok);
          std::getline(ss, tok, ','); a.sy = std::stoi(tok);
          std::getline(ss, tok, ','); a.sh = parse_heading(tok); }
        { std::istringstream ss(gline);
          std::getline(ss, tok, ','); a.gx = std::stoi(tok);
          std::getline(ss, tok, ','); a.gy = std::stoi(tok); }
        m.agents.push_back(a);
    }

    read_tag('M');
    m.grid.assign(m.W, std::vector<int>(m.H, 0));
    for (int y = 0; y < m.H; ++y) {
        std::getline(f, line);
        std::istringstream ss(line);
        for (int x = 0; x < m.W; ++x) {
            std::getline(ss, tok, ',');
            m.grid[x][y] = std::stoi(tok);
        }
    }
    std::cout << "Loaded map " << m.W << "x" << m.H
              << "  agents=" << m.agents.size()
              << "  (file had " << n_agents << ")\n";
    return m;
}

// ── BFS heuristic ─────────────────────────────────────────────────────────────
// Precompute BFS distance from (gx,gy) to every reachable (x,y) cell.
// Ignores heading — gives an admissible lower bound on move-actions needed.

static std::vector<std::vector<int>> bfs_heuristic(const Map& map, int gx, int gy) {
    std::vector<std::vector<int>> dist(map.W, std::vector<int>(map.H, INT_MAX));
    std::queue<std::pair<int,int>> q;
    dist[gx][gy] = 0;
    q.push({gx, gy});
    while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        for (int d = 0; d < 4; ++d) {
            int nx = cx + DX[d], ny = cy + DY[d];
            if (map.passable(nx, ny) && dist[nx][ny] == INT_MAX) {
                dist[nx][ny] = dist[cx][cy] + 1;
                q.push({nx, ny});
            }
        }
    }
    return dist;
}

// ── Low-level A* ─────────────────────────────────────────────────────────────
// State: (x, y, heading, t)
// Search from (sx,sy,sh,0) to (gx,gy,*,t>=0) under constraints.

struct AStarNode {
    int f, g, x, y, h, t;
    bool operator>(const AStarNode& o) const { return f > o.f; }
};

// Key for closed set: (x, y, h, t) packed into 64 bits
static inline long long encode(int x, int y, int h, int t) {
    return ((long long)t << 20) | ((long long)x << 10) | ((long long)y << 4) | h;
}

static Path low_level_astar(const Map& map, const AgentDef& agent,
                             const Constraints& cons, int agent_id,
                             const std::vector<std::vector<int>>& hdist) {
    // Build fast lookup sets for constraints
    std::set<std::tuple<int,int,int>>         vc;   // (x,y,t)
    std::set<std::tuple<int,int,int,int,int>> ec;   // (x1,y1,x2,y2,t)
    for (auto& c : cons.vert) if (c.agent == agent_id) vc.insert({c.x, c.y, c.t});
    for (auto& c : cons.edge) if (c.agent == agent_id) ec.insert({c.x1,c.y1,c.x2,c.y2,c.t});

    int gx = agent.gx, gy = agent.gy;

    // h value: BFS distance, or 0 if goal unreachable (handles edge cases)
    auto hval = [&](int x, int y) -> int {
        if (hdist[x][y] == INT_MAX) return 0;
        return hdist[x][y];
    };

    std::unordered_map<long long, int>      gcost;
    std::unordered_map<long long, long long> parent;
    struct NodeInfo { int x, y, h, t; };
    std::unordered_map<long long, NodeInfo>  ndata;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;

    int sx = agent.sx, sy = agent.sy, sh = agent.sh;
    long long sk = encode(sx, sy, sh, 0);
    int hv = hval(sx, sy);
    open.push({hv, 0, sx, sy, sh, 0});
    gcost[sk] = 0;
    parent[sk] = -1;
    ndata[sk]  = {sx, sy, sh, 0};

    long long goal_key = -1;

    while (!open.empty()) {
        AStarNode cur = open.top(); open.pop();
        long long ck = encode(cur.x, cur.y, cur.h, cur.t);

        auto it = gcost.find(ck);
        if (it == gcost.end() || it->second < cur.g) continue;

        if (cur.x == gx && cur.y == gy) { goal_key = ck; break; }
        if (cur.t >= MAX_TIMESTEP) continue;

        int nt = cur.t + 1;

        // 5 actions: forward, backward, turn-left, turn-right, wait
        for (int act = 0; act < 5; ++act) {
            int nx = cur.x, ny = cur.y, nh = cur.h;
            bool moves = false;
            if      (act == 0) { nx+=DX[cur.h]; ny+=DY[cur.h]; moves=true; }
            else if (act == 1) { nx-=DX[cur.h]; ny-=DY[cur.h]; moves=true; }
            else if (act == 2) { nh=(cur.h+1)%4; }
            else if (act == 3) { nh=(cur.h+3)%4; }
            // act==4: wait (nx,ny,nh unchanged)

            if (!map.passable(nx, ny)) continue;
            if (vc.count({nx, ny, nt})) continue;
            if (moves && ec.count({cur.x,cur.y,nx,ny,cur.t})) continue;

            int ng  = cur.g + 1;
            long long nk = encode(nx, ny, nh, nt);
            auto git = gcost.find(nk);
            if (git != gcost.end() && git->second <= ng) continue;

            gcost[nk]  = ng;
            parent[nk] = ck;
            ndata[nk]  = {nx, ny, nh, nt};
            open.push({ng + hval(nx, ny), ng, nx, ny, nh, nt});
        }
    }

    if (goal_key == -1) return {};

    // Reconstruct
    std::vector<NodeInfo> rev;
    long long k = goal_key;
    while (k != -1LL) {
        rev.push_back(ndata[k]);
        auto pit = parent.find(k);
        if (pit == parent.end() || pit->second == -1LL) break;
        k = pit->second;
    }
    std::reverse(rev.begin(), rev.end());

    Path path;
    for (auto& nd : rev) path.push_back({nd.t, nd.x, nd.y, nd.h});
    return path;
}

// ── Conflict detection ────────────────────────────────────────────────────────

struct Conflict {
    enum Type { VERTEX, EDGE } type;
    int a1, a2, t;
    int x, y;           // vertex
    int x1, y1, x2, y2; // edge (a1 direction)
};

static Step path_at(const Path& p, int t) {
    if (p.empty()) return {t, -1, -1, 0};
    return (t < (int)p.size()) ? p[t] : p.back();
}

// Returns the first conflict found, or empty vector if none.
static std::vector<Conflict> find_conflicts(const std::vector<Path>& paths) {
    int n = (int)paths.size();
    int T = 0;
    for (auto& p : paths) T = std::max(T, (int)p.size());
    T = std::min(T + 1, MAX_TIMESTEP);

    for (int t = 0; t < T; ++t) {
        for (int i = 0; i < n; ++i) {
            Step si = path_at(paths[i], t);
            for (int j = i+1; j < n; ++j) {
                Step sj = path_at(paths[j], t);
                if (si.x == sj.x && si.y == sj.y)
                    return {{Conflict::VERTEX, i, j, t, si.x, si.y, 0,0,0,0}};
                if (t+1 < T) {
                    Step si2 = path_at(paths[i], t+1);
                    Step sj2 = path_at(paths[j], t+1);
                    if (si.x==sj2.x && si.y==sj2.y && sj.x==si2.x && sj.y==si2.y)
                        return {{Conflict::EDGE, i, j, t, 0,0, si.x,si.y,sj.x,sj.y}};
                }
            }
        }
    }
    return {};
}

// ── CBS ───────────────────────────────────────────────────────────────────────

struct CTNode {
    Constraints         cons;
    std::vector<Path>   paths;
    int                 cost;
};

static int sum_cost(const std::vector<Path>& paths) {
    int c = 0;
    for (auto& p : paths) c += (int)p.size();
    return c;
}

struct CTCmp {
    bool operator()(const CTNode* a, const CTNode* b) { return a->cost > b->cost; }
};

static std::vector<Path> cbs(const Map& map) {
    int n = (int)map.agents.size();
    std::cout << "CBS: planning for " << n << " agents\n";

    // Precompute BFS heuristics for each agent
    std::vector<std::vector<std::vector<int>>> h_tables(n);
    for (int i = 0; i < n; ++i)
        h_tables[i] = bfs_heuristic(map, map.agents[i].gx, map.agents[i].gy);

    // Root node
    auto* root = new CTNode();
    for (int i = 0; i < n; ++i) {
        Path p = low_level_astar(map, map.agents[i], root->cons, i, h_tables[i]);
        if (p.empty()) {
            std::cerr << "  Agent " << i << " has no individual path — skipping.\n";
            p.push_back({0, map.agents[i].sx, map.agents[i].sy, map.agents[i].sh});
        }
        root->paths.push_back(p);
    }
    root->cost = sum_cost(root->paths);

    std::priority_queue<CTNode*, std::vector<CTNode*>, CTCmp> open;
    open.push(root);

    int nodes_expanded = 0;
    auto t0 = std::chrono::steady_clock::now();

    while (!open.empty()) {
        CTNode* cur = open.top(); open.pop();
        ++nodes_expanded;

        if (nodes_expanded % 200 == 0) {
            double sec = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t0).count();
            std::cout << "  CBS nodes=" << nodes_expanded
                      << "  cost=" << cur->cost << "  t=" << sec << "s\n";
        }
        if (nodes_expanded > MAX_CBS_NODES) {
            std::cout << "CBS: node limit reached — returning best-effort solution\n";
            auto result = cur->paths;
            delete cur;
            while (!open.empty()) { delete open.top(); open.pop(); }
            return result;
        }

        auto conflicts = find_conflicts(cur->paths);
        if (conflicts.empty()) {
            std::cout << "CBS: optimal solution found after " << nodes_expanded << " nodes\n";
            auto result = cur->paths;
            delete cur;
            while (!open.empty()) { delete open.top(); open.pop(); }
            return result;
        }

        const Conflict& c = conflicts[0];
        for (int branch = 0; branch < 2; ++branch) {
            int ag = (branch == 0) ? c.a1 : c.a2;
            auto* child = new CTNode();
            child->cons  = cur->cons;
            child->paths = cur->paths;

            if (c.type == Conflict::VERTEX) {
                child->cons.vert.push_back({ag, c.x, c.y, c.t});
            } else {
                if (ag == c.a1)
                    child->cons.edge.push_back({ag, c.x1,c.y1,c.x2,c.y2, c.t});
                else
                    child->cons.edge.push_back({ag, c.x2,c.y2,c.x1,c.y1, c.t});
            }

            Path p = low_level_astar(map, map.agents[ag], child->cons, ag, h_tables[ag]);
            if (p.empty()) { delete child; continue; }
            child->paths[ag] = p;
            child->cost = sum_cost(child->paths);
            open.push(child);
        }
        delete cur;
    }

    std::cerr << "CBS: no solution found.\n";
    return {};
}

// ── Space-time A* for Prioritized Planning ───────────────────────────────────
// Like low_level_astar but checks a shared reservation table instead of
// a per-agent Constraints struct.  The reservation table maps (x,y,t) → bool.

using ReservedSet = std::set<std::tuple<int,int,int>>;   // (x, y, t)

static Path pp_astar(const Map& map, const AgentDef& agent,
                     const ReservedSet& reserved,
                     const std::vector<std::vector<int>>& hdist) {
    int gx = agent.gx, gy = agent.gy;
    auto hval = [&](int x, int y) -> int {
        return (hdist[x][y] == INT_MAX) ? 0 : hdist[x][y];
    };

    std::unordered_map<long long, int>       gcost;
    std::unordered_map<long long, long long>  parent;
    struct NI { int x, y, h, t; };
    std::unordered_map<long long, NI>         ndata;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    int sx=agent.sx, sy=agent.sy, sh=agent.sh;
    long long sk = encode(sx,sy,sh,0);
    open.push({hval(sx,sy), 0, sx, sy, sh, 0});
    gcost[sk]=0; parent[sk]=-1; ndata[sk]={sx,sy,sh,0};

    long long goal_key = -1;
    while (!open.empty()) {
        AStarNode cur = open.top(); open.pop();
        long long ck = encode(cur.x,cur.y,cur.h,cur.t);
        if (gcost.count(ck) && gcost[ck] < cur.g) continue;

        if (cur.x==gx && cur.y==gy) { goal_key=ck; break; }
        if (cur.t >= MAX_TIMESTEP) continue;

        int nt = cur.t+1;
        for (int act = 0; act < 5; ++act) {
            int nx=cur.x, ny=cur.y, nh=cur.h;
            bool moves=false;
            if      (act==0){nx+=DX[cur.h];ny+=DY[cur.h];moves=true;}
            else if (act==1){nx-=DX[cur.h];ny-=DY[cur.h];moves=true;}
            else if (act==2){nh=(cur.h+1)%4;}
            else if (act==3){nh=(cur.h+3)%4;}

            if (!map.passable(nx,ny)) continue;
            if (reserved.count({nx,ny,nt})) continue;
            // Swap conflict: if agent moves (x,y)->(nx,ny) and reserved cell has (nx,ny,t) AND (x,y,nt)
            if (moves && reserved.count({cur.x,cur.y,nt}) && reserved.count({nx,ny,cur.t})) continue;

            int ng = cur.g+1;
            long long nk = encode(nx,ny,nh,nt);
            if (gcost.count(nk) && gcost[nk]<=ng) continue;
            gcost[nk]=ng; parent[nk]=ck; ndata[nk]={nx,ny,nh,nt};
            open.push({ng+hval(nx,ny), ng, nx, ny, nh, nt});
        }
    }
    if (goal_key==-1) return {};
    std::vector<NI> rev;
    for (long long k=goal_key; k!=-1LL; ) {
        rev.push_back(ndata[k]);
        auto pit=parent.find(k);
        if (pit==parent.end()||pit->second==-1LL) break;
        k=pit->second;
    }
    std::reverse(rev.begin(),rev.end());
    Path path;
    for (auto& nd:rev) path.push_back({nd.t,nd.x,nd.y,nd.h});
    return path;
}

// ── Prioritized Planning ──────────────────────────────────────────────────────

static std::vector<Path> prioritized_planning(const Map& map) {
    int n = (int)map.agents.size();
    std::cout << "Prioritized Planning: " << n << " agents\n";

    // Priority: plan the hardest agents (longest unconstrained BFS path) first.
    // This prevents easy-to-plan agents from blocking the harder ones.
    std::vector<int> bfs_dist(n);
    for (int i = 0; i < n; ++i) {
        auto hd = bfs_heuristic(map, map.agents[i].gx, map.agents[i].gy);
        int d = hd[map.agents[i].sx][map.agents[i].sy];
        bfs_dist[i] = (d == INT_MAX) ? 0 : d;
    }
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return bfs_dist[a] > bfs_dist[b];   // harder (longer path) → higher priority
    });

    ReservedSet reserved;  // (x, y, t) cells already taken
    std::vector<Path> paths(n);

    for (int idx = 0; idx < n; ++idx) {
        int i = order[idx];
        auto hdist = bfs_heuristic(map, map.agents[i].gx, map.agents[i].gy);
        Path p = pp_astar(map, map.agents[i], reserved, hdist);

        if (p.empty()) {
            std::cerr << "  Agent " << i << " has no path — staying at start.\n";
            p.push_back({0, map.agents[i].sx, map.agents[i].sy, map.agents[i].sh});
        }
        paths[i] = p;
        if (idx % 10 == 0) std::cout << "  Planned " << idx+1 << "/" << n << "\n";

        // Reserve all timesteps in this path
        for (auto& s : p) reserved.insert({s.x, s.y, s.t});
        // Reserve goal cell for all future times (parked agent stays there)
        if (!p.empty()) {
            auto& last = p.back();
            for (int t = last.t+1; t <= MAX_TIMESTEP; ++t)
                reserved.insert({last.x, last.y, t});
        }
    }
    std::cout << "Prioritized Planning: done.\n";
    return paths;
}

// ── Output ────────────────────────────────────────────────────────────────────

static void write_trajectories(const std::vector<Path>& paths, const std::string& out_path) {
    std::string dir = out_path;
    auto pos = dir.rfind('/');
    if (pos != std::string::npos) {
        dir = dir.substr(0, pos);
        (void)std::system(("mkdir -p \"" + dir + "\"").c_str());
    }
    std::ofstream f(out_path);
    if (!f) { std::cerr << "Cannot write: " << out_path << "\n"; return; }
    f << "# agent_id,timestep,x,y,heading  (0=E 1=N 2=W 3=S)\n";
    for (int i = 0; i < (int)paths.size(); ++i)
        for (auto& s : paths[i])
            f << i << "," << s.t << "," << s.x << "," << s.y << "," << s.h << "\n";
    std::cout << "Trajectories written: " << out_path << "\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::string map_path = "map/parking_lot.txt";
    std::string out_path = "output/trajectories.txt";
    int  max_agents = INT_MAX;
    bool use_pp     = false;
    int  positional = 0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--max-agents" && i+1 < argc) max_agents = std::stoi(argv[++i]);
        else if (a == "--pp")                        use_pp = true;
        else if (a[0] != '-') {
            if (positional == 0) map_path = a;
            else                 out_path = a;
            ++positional;
        }
    }

    Map map = load_map(map_path, max_agents);
    if (map.agents.empty()) { std::cerr << "No agents.\n"; return 1; }

    std::vector<Path> paths;
    if (use_pp)
        paths = prioritized_planning(map);
    else
        paths = cbs(map);

    if (paths.empty()) { std::cerr << "No solution.\n"; return 1; }

    write_trajectories(paths, out_path);

    int max_t = 0;
    for (auto& p : paths) if (!p.empty()) max_t = std::max(max_t, p.back().t);
    std::cout << "Makespan: " << max_t << "  agents: " << paths.size() << "\n";
    return 0;
}
