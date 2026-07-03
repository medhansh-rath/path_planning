# Path Planning Benchmark

A C++ benchmark scaffold for comparing grid-based planners on Phobos-like terrain.

## Goals

- Compare planners such as Field D* and D* Lite on grid sizes from 100x100 to 800x800.
- Track planning time, replanning time, memory usage, path length, node expansions, and total traversal cost.
- Generate seeded terrain with smooth slope gradients and small obstacle clusters.
- Optionally export visualization frames so rover motion can be reviewed after a run.
- Support dynamic obstacle injection during execution to stress replanning.

## Current structure

The repository is organized around a virtual planner interface so new algorithms can be dropped in without changing the benchmark harness.

- `path_planning::Planner` defines the common planner contract.
- `path_planning::BenchmarkRunner` drives scenarios and collects metrics.
- `path_planning::PhobosTerrainGenerator` builds seeded terrain with slope-like traversal costs and obstacles.
- `path_planning::visualization::PpmSequenceVisualizer` exports frames when visualization is enabled.
- `path_planning::FieldDStarPlanner` is present as the future implementation slot for the Field D* algorithm.

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/path_planning_benchmark --sizes=100,200,300,400,500,600,700,800 --seed=42 --visualize --output=artifacts/frames
```

## Notes

- The benchmark harness is ready for Field D* and D* Lite implementations.
- The planner interface is intentionally virtual so future algorithms can be added with minimal coupling.
- Visualization uses PPM frame export to keep the repository dependency-light.
- D* Lite is wired into the default planner list as a runnable incremental replanning reference.
