# Path Planning Benchmark

A C++ benchmark scaffold for comparing grid-based planners on Phobos-like terrain.

## Goals

- Compare planners such as Field D* and D* Lite on grid sizes from 100x100 to 800x800.
- Track planning time, replanning time, memory usage, path length, node expansions, and total traversal cost.
- Track planning time, replanning time, memory usage, path length, path turns, and path efficiency.
- Generate seeded terrain with smooth slope gradients and small obstacle clusters.
- Optionally export visualization frames so rover motion can be reviewed after a run.
- Support dynamic obstacle injection during execution to stress replanning.

## Current structure

The repository is organized around a virtual planner interface so new algorithms can be dropped in without changing the benchmark harness.

- `path_planning::Planner` defines the common planner contract.
- `path_planning::BenchmarkRunner` drives scenarios and collects metrics.
- `path_planning::PhobosTerrainGenerator` builds seeded terrain with slope-like traversal costs and obstacles.
- `path_planning::visualization::PpmSequenceVisualizer` exports frames when visualization is enabled.
- `path_planning::FieldDStarPlanner` provides the Field D* style any-angle planner used in the default comparison.

## Scripts

- `scripts/render_rover_motion.py` turns a frame directory into a rover-motion video with `ffmpeg`.
- `scripts/compare_benchmarks.py` reads benchmark CSV output and writes SVG comparison plots plus a compact HTML index.
- The comparison plots now include path-efficiency and turn-count views in addition to timing and cost metrics.

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/path_planning_benchmark --sizes=100,200,300,400,500,600,700,800 --seed=42 --visualize --output=artifacts/frames
```

To render the rover motion into a video after a run:

```bash
python3 scripts/render_rover_motion.py --input artifacts/frames/field_dstar/100x100 --output artifacts/field_dstar_100.mp4
```

To generate benchmark comparison charts:

```bash
python3 scripts/compare_benchmarks.py --input benchmark_results.csv --output artifacts/plots
```

## Notes

- The benchmark harness is ready for Field D* and D* Lite implementations.
- The planner interface is intentionally virtual so future algorithms can be added with minimal coupling.
- Visualization uses PPM frame export to keep the repository dependency-light.
- The default benchmark set compares Field D* against D* Lite.
