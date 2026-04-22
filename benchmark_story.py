#!/usr/bin/env python3
"""
Generate a benchmark/process visual for the Gauntlet stress scenario.

The figure is designed for the final project write-up: it benchmarks CBS
against Prioritized Planning on balanced crossing cases and explains why the
Gauntlet bottleneck triggers combinatorial blow-up for CBS.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

_CACHE_DIR = Path(tempfile.gettempdir()) / "multi-agent-planning-cache"
(_CACHE_DIR / "matplotlib").mkdir(parents=True, exist_ok=True)
(_CACHE_DIR / "xdg").mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_CACHE_DIR / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(_CACHE_DIR / "xdg"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

import visualizer


CBS_COLOR = "#d64f4f"
PP_COLOR = "#2b7de9"
BOTTLENECK_COLOR = "#ffcc66"
TEXT_DARK = "#1f2933"
MUTED = "#5f6c7b"
PANEL_BG = "#f7f8fa"
GRID_COLOR = "#d8dee6"

DEFAULT_COUNTS = [4, 8, 12, 16, 20, 24]


@dataclass
class BenchmarkResult:
    algorithm: str
    agents: int
    status: str
    runtime_sec: float
    planned_agents: int
    skipped_agents: int
    makespan: int
    sum_of_costs: int
    low_level_searches: int
    passes: int
    pass1_stuck: int
    nodes_expanded: int
    root_vertex_conflicts: int
    root_edge_conflicts: int
    root_conflicts: int
    node_limit_hit: bool
    stationary_steps: int


def load_module(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load module from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def balanced_gauntlet_agents(gauntlet_module, total_agents: int):
    if total_agents % 2 != 0:
        raise ValueError("Balanced gauntlet counts must be even.")
    all_agents = gauntlet_module.all_crossing_agents()
    half = total_agents // 2
    top_agents = all_agents[:12][:half]
    bottom_agents = all_agents[12:][:half]
    return top_agents + bottom_agents


def parse_stat_line(output: str) -> dict[str, str]:
    stat_line = None
    for line in output.splitlines():
        if line.startswith("STAT "):
            stat_line = line
            break
    if stat_line is None:
        raise RuntimeError("Planner output did not contain a STAT line.")
    return dict(re.findall(r"(\w+)=([^\s]+)", stat_line))


def parse_stationary_steps(map_path: Path, traj_path: Path) -> int:
    map_data = visualizer.parse_map(str(map_path))
    trajectories = visualizer.parse_trajectories(str(traj_path), map_data["num_agents"])
    stationary_steps = 0
    for traj in trajectories:
        if len(traj) < 2:
            continue
        for prev, cur in zip(traj[:-1], traj[1:]):
            if int(prev[1]) == int(cur[1]) and int(prev[2]) == int(cur[2]):
                stationary_steps += 1
    return stationary_steps


def run_planner(
    repo_root: Path,
    planner_path: Path,
    map_path: Path,
    traj_path: Path,
    use_pp: bool,
) -> BenchmarkResult:
    cmd = [str(planner_path)]
    if use_pp:
        cmd.append("--pp")
    cmd.extend([str(map_path), str(traj_path)])
    proc = subprocess.run(
        cmd,
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=True,
    )
    combined = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    stats = parse_stat_line(combined)
    stationary_steps = parse_stationary_steps(map_path, traj_path)
    return BenchmarkResult(
        algorithm=stats["algorithm"],
        agents=int(stats["planned_agents"]) + int(stats["skipped_agents"]),
        status=stats["status"],
        runtime_sec=float(stats["runtime_sec"]),
        planned_agents=int(stats["planned_agents"]),
        skipped_agents=int(stats["skipped_agents"]),
        makespan=int(stats["makespan"]),
        sum_of_costs=int(stats["sum_of_costs"]),
        low_level_searches=int(stats["low_level_searches"]),
        passes=int(stats["passes"]),
        pass1_stuck=int(stats["pass1_stuck"]),
        nodes_expanded=int(stats["nodes_expanded"]),
        root_vertex_conflicts=int(stats["root_vertex_conflicts"]),
        root_edge_conflicts=int(stats["root_edge_conflicts"]),
        root_conflicts=int(stats["root_conflicts"]),
        node_limit_hit=bool(int(stats["node_limit_hit"])),
        stationary_steps=stationary_steps,
    )


def benchmark_balanced_gauntlet(repo_root: Path, planner_path: Path, counts: list[int]):
    gauntlet_module = load_module("generate_gauntlet", repo_root / "map" / "generate_gauntlet.py")
    results: list[BenchmarkResult] = []
    reference_map = None

    with tempfile.TemporaryDirectory(prefix="gauntlet_benchmark_") as tmpdir:
        tmpdir_path = Path(tmpdir)
        for total_agents in counts:
            agents = balanced_gauntlet_agents(gauntlet_module, total_agents)
            map_path = tmpdir_path / f"gauntlet_{total_agents}.txt"
            traj_cbs = tmpdir_path / f"gauntlet_{total_agents}_cbs.txt"
            traj_pp = tmpdir_path / f"gauntlet_{total_agents}_pp.txt"

            gauntlet_module.write_map(gauntlet_module.generate(), str(map_path), agents)
            if total_agents == max(counts):
                reference_map = map_path

            results.append(run_planner(repo_root, planner_path, map_path, traj_cbs, use_pp=False))
            results.append(run_planner(repo_root, planner_path, map_path, traj_pp, use_pp=True))

        if reference_map is None:
            raise RuntimeError("Failed to produce a reference gauntlet map.")
        reference_copy = tmpdir_path / "gauntlet_reference.txt"
        reference_copy.write_text(reference_map.read_text())
        reference_map = reference_copy

        results.sort(key=lambda r: (r.agents, r.algorithm))
        payload = {
            "counts": counts,
            "results": [asdict(r) for r in results],
            "reference_map_text": reference_map.read_text(),
        }
        return results, payload


def load_reference_map(map_text: str, target_path: Path) -> dict:
    target_path.write_text(map_text)
    return visualizer.parse_map(str(target_path))


def grouped(results: list[BenchmarkResult], algorithm: str) -> list[BenchmarkResult]:
    return [r for r in results if r.algorithm == algorithm]


def add_chart_style(ax):
    ax.set_facecolor("white")
    ax.grid(True, color=GRID_COLOR, alpha=0.7, linewidth=0.8)
    for spine in ax.spines.values():
        spine.set_color("#c2cad3")
    ax.tick_params(colors=TEXT_DARK)
    ax.title.set_color(TEXT_DARK)
    ax.xaxis.label.set_color(TEXT_DARK)
    ax.yaxis.label.set_color(TEXT_DARK)


def draw_map_panel(ax, map_data: dict, cbs_last: BenchmarkResult, pp_last: BenchmarkResult):
    img = visualizer._build_rgb(map_data["costmap"], map_data["collision_thresh"])
    ax.imshow(img, origin="lower", interpolation="nearest")
    visualizer._add_stall_lines(ax, map_data["costmap"])
    ax.set_facecolor(PANEL_BG)

    # Central bottleneck: x = 4..11, y = 11..12 in the Gauntlet map.
    rect = Rectangle(
        (3.5, 10.5),
        8.0,
        2.0,
        linewidth=2.8,
        edgecolor=BOTTLENECK_COLOR,
        facecolor=BOTTLENECK_COLOR,
        alpha=0.22,
        zorder=5,
    )
    ax.add_patch(rect)

    arrow_kw = dict(arrowstyle="simple", mutation_scale=36, linewidth=0, alpha=0.45, zorder=6)
    ax.add_patch(FancyArrowPatch((6.2, 20.9), (6.2, 8.4), color=CBS_COLOR, **arrow_kw))
    ax.add_patch(FancyArrowPatch((9.3, 2.2), (9.3, 15.5), color=PP_COLOR, **arrow_kw))

    outline = [pe.withStroke(linewidth=3, foreground="white")]
    ax.text(
        7.5,
        12.9,
        "2-cell central lane\nall opposing flows meet here",
        ha="center",
        va="bottom",
        fontsize=11,
        color=TEXT_DARK,
        weight="bold",
        path_effects=outline,
        zorder=7,
    )
    ax.text(
        6.2,
        21.6,
        "top queue\nagents head south",
        ha="center",
        va="bottom",
        fontsize=10,
        color=TEXT_DARK,
        path_effects=outline,
        zorder=7,
    )
    ax.text(
        9.3,
        1.4,
        "bottom queue\nagents head north",
        ha="center",
        va="top",
        fontsize=10,
        color=TEXT_DARK,
        path_effects=outline,
        zorder=7,
    )

    ax.text(
        0.02,
        0.98,
        (
            "Gauntlet stress test\n"
            f"{pp_last.agents} agents, balanced across both sides\n"
            f"CBS root conflicts at {cbs_last.agents} agents: {cbs_last.root_conflicts}"
        ),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=11,
        color=TEXT_DARK,
        bbox=dict(boxstyle="round,pad=0.45", facecolor="white", edgecolor="#d4dbe3", alpha=0.95),
        zorder=8,
    )

    ax.set_xlim(-0.5, map_data["x_size"] - 0.5)
    ax.set_ylim(-0.5, map_data["y_size"] - 0.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Why This Scenario Punishes Coordination", fontsize=15, weight="bold", pad=10)


def plot_runtime(ax, cbs: list[BenchmarkResult], pp: list[BenchmarkResult]):
    add_chart_style(ax)
    counts = [r.agents for r in cbs]
    cbs_rt = [r.runtime_sec for r in cbs]
    pp_rt = [r.runtime_sec for r in pp]

    ax.plot(counts, cbs_rt, color=CBS_COLOR, marker="o", linewidth=2.5, label="CBS")
    ax.plot(counts, pp_rt, color=PP_COLOR, marker="o", linewidth=2.5, label="Prioritized Planning")
    ax.set_yscale("log")
    ax.set_xlabel("Agents")
    ax.set_ylabel("Runtime (s, log scale)")
    ax.set_title("Runtime Scaling")
    ax.legend(loc="upper left", frameon=False)

    capped = [r for r in cbs if r.node_limit_hit]
    for result in capped:
        ax.scatter(result.agents, result.runtime_sec, color=CBS_COLOR, marker="x", s=120, zorder=5)
    if capped:
        first = capped[0]
        ax.annotate(
            "10k-node cap\nfrom 12+ agents",
            xy=(first.agents, first.runtime_sec),
            xytext=(first.agents + 2.0, first.runtime_sec * 0.55),
            arrowprops=dict(arrowstyle="->", color=CBS_COLOR, lw=1.2),
            fontsize=9,
            color=CBS_COLOR,
        )


def plot_searches(ax, cbs: list[BenchmarkResult], pp: list[BenchmarkResult]):
    add_chart_style(ax)
    counts = [r.agents for r in cbs]
    ax.plot(
        counts,
        [r.low_level_searches for r in cbs],
        color=CBS_COLOR,
        marker="o",
        linewidth=2.5,
        label="CBS low-level A* calls",
    )
    ax.plot(
        counts,
        [r.low_level_searches for r in pp],
        color=PP_COLOR,
        marker="o",
        linewidth=2.5,
        label="PP low-level A* calls",
    )
    ax.set_yscale("log")
    ax.set_xlabel("Agents")
    ax.set_ylabel("A* calls (log scale)")
    ax.set_title("Search Work")
    ax.legend(loc="upper left", frameon=False)


def add_process_box(ax, xy, width, height, title, color, lines):
    box = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.02",
        facecolor="white",
        edgecolor=color,
        linewidth=2.0,
    )
    ax.add_patch(box)
    x0, y0 = xy
    ax.text(x0 + 0.04 * width, y0 + height - 0.12 * height, title, color=color, fontsize=14, weight="bold")
    body = "\n".join(lines)
    ax.text(
        x0 + 0.04 * width,
        y0 + height - 0.22 * height,
        body,
        color=TEXT_DARK,
        fontsize=9.4,
        va="top",
        linespacing=1.45,
    )


def draw_process_panel(ax, cbs_last: BenchmarkResult, cbs_best: BenchmarkResult, pp_last: BenchmarkResult):
    ax.set_facecolor(PANEL_BG)
    ax.axis("off")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title("Planning Process: Why CBS Blows Up Here", fontsize=15, weight="bold", pad=10)

    add_process_box(
        ax,
        (0.02, 0.08),
        0.45,
        0.78,
        "CBS",
        CBS_COLOR,
        [
            "1. Plan each agent independently.",
            f"2. Root shortest paths: {cbs_last.root_conflicts} conflicts at {cbs_last.agents} agents.",
            "3. Each conflict creates two CT children.",
            "4. Every child re-runs A* for one agent.",
            "",
            f"{cbs_last.agents} agents: {cbs_last.nodes_expanded:,} CT nodes",
            f"{cbs_last.low_level_searches:,} low-level A* calls",
            f"{cbs_last.runtime_sec:.2f}s before the node cap stops search",
        ],
    )

    add_process_box(
        ax,
        (0.53, 0.08),
        0.45,
        0.78,
        "Prioritized Planning",
        PP_COLOR,
        [
            "1. Plan the front of each queue first.",
            "2. Reserve every committed (x, y, t) cell.",
            "3. Later agents wait or detour; no CT branching.",
            "4. Re-plan only if pass 1 leaves agents stuck.",
            "",
            f"{pp_last.agents} agents: {pp_last.low_level_searches} low-level A* calls",
            f"{pp_last.runtime_sec:.3f}s runtime, makespan {pp_last.makespan}",
            f"Queueing cost: {pp_last.stationary_steps} stationary timesteps",
        ],
    )

    ax.text(
        0.5,
        0.94,
        (
            f"Last optimal CBS point: {cbs_best.agents} agents "
            f"({cbs_best.nodes_expanded:,} CT nodes, {cbs_best.runtime_sec:.2f}s)"
        ),
        ha="center",
        va="center",
        fontsize=10.5,
        color=MUTED,
    )


def render_figure(results: list[BenchmarkResult], map_data: dict, output_path: Path):
    cbs = grouped(results, "CBS")
    pp = grouped(results, "PP")
    cbs_last = cbs[-1]
    pp_last = pp[-1]
    cbs_optimal = [r for r in cbs if r.status == "optimal"]
    cbs_best = cbs_optimal[-1] if cbs_optimal else cbs[-1]

    fig = plt.figure(figsize=(17, 10), facecolor=PANEL_BG)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.35, 1.0, 1.0], hspace=0.28, wspace=0.22)

    ax_map = fig.add_subplot(gs[:, 0])
    ax_runtime = fig.add_subplot(gs[0, 1])
    ax_searches = fig.add_subplot(gs[0, 2])
    ax_process = fig.add_subplot(gs[1, 1:])

    draw_map_panel(ax_map, map_data, cbs_last, pp_last)
    plot_runtime(ax_runtime, cbs, pp)
    plot_searches(ax_searches, cbs, pp)
    draw_process_panel(ax_process, cbs_last, cbs_best, pp_last)

    fig.suptitle(
        "Gauntlet Benchmark Story: CBS Optimizes Through Conflicts, PP Schedules Around Them",
        fontsize=18,
        weight="bold",
        color=TEXT_DARK,
        y=0.98,
    )
    fig.text(
        0.5,
        0.945,
        "Balanced crossing cases on the 16x24 Gauntlet map, benchmarked from the current planner implementation.",
        ha="center",
        fontsize=11,
        color=MUTED,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def prepare_planner(repo_root: Path, requested_planner: Path):
    source_path = repo_root / "planner.cpp"
    default_repo_planner = (repo_root / "planner").resolve()
    always_rebuild = requested_planner.resolve() == default_repo_planner
    needs_rebuild = (
        always_rebuild
        or not requested_planner.exists()
        or source_path.stat().st_mtime > requested_planner.stat().st_mtime
    )
    if not needs_rebuild:
        return requested_planner, None

    build_dir = tempfile.TemporaryDirectory(prefix="planner_build_")
    temp_planner = Path(build_dir.name) / requested_planner.name
    print(f"Building fresh planner binary at {temp_planner}")
    subprocess.run(
        ["g++", "-O2", "-std=c++17", "-o", str(temp_planner), str(source_path)],
        cwd=repo_root,
        check=True,
    )
    return temp_planner, build_dir


def main():
    parser = argparse.ArgumentParser(description="Generate a benchmark/process visual for the Gauntlet scenario.")
    parser.add_argument(
        "--planner",
        default="planner",
        help="Path to the planner binary relative to the repo root (default: planner).",
    )
    parser.add_argument(
        "--output",
        default="docs/benchmark_story.png",
        help="Where to save the rendered figure (default: docs/benchmark_story.png).",
    )
    parser.add_argument(
        "--metrics",
        default="docs/benchmark_metrics.json",
        help="Where to save the raw benchmark metrics (default: docs/benchmark_metrics.json).",
    )
    parser.add_argument(
        "--counts",
        nargs="+",
        type=int,
        default=DEFAULT_COUNTS,
        help="Balanced Gauntlet agent counts to benchmark (default: 4 8 12 16 20 24).",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent
    planner_path = (repo_root / args.planner).resolve()
    output_path = (repo_root / args.output).resolve()
    metrics_path = (repo_root / args.metrics).resolve()
    planner_path, build_dir = prepare_planner(repo_root, planner_path)

    try:
        results, payload = benchmark_balanced_gauntlet(repo_root, planner_path, args.counts)

        metrics_path.parent.mkdir(parents=True, exist_ok=True)
        metrics_path.write_text(json.dumps(payload, indent=2))

        reference_map_path = metrics_path.parent / "_benchmark_story_reference_map.txt"
        map_data = load_reference_map(payload["reference_map_text"], reference_map_path)
        render_figure(results, map_data, output_path)
        reference_map_path.unlink(missing_ok=True)

        print(f"Saved figure: {output_path}")
        print(f"Saved metrics: {metrics_path}")
    finally:
        if build_dir is not None:
            build_dir.cleanup()


if __name__ == "__main__":
    main()
