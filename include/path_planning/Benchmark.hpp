#pragma once

#include "path_planning/PhobosTerrainGenerator.hpp"
#include "path_planning/planners/AStarPlanner.hpp"
#include "path_planning/planners/DStarLitePlanner.hpp"
#include "path_planning/planners/FieldDStarPlanner.hpp"
#include "path_planning/visualization/PpmSequenceVisualizer.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef __linux__
#include <sys/resource.h>
#endif

namespace path_planning {

struct BenchmarkConfig {
  std::vector<int> sizes{100, 200, 300, 400, 500, 600, 700, 800};
  std::uint32_t seed = 42;
  bool visualize = false;
  std::string visualizationDirectory = "artifacts/frames";
  int visualizationScale = 1;
  std::size_t visualizationStride = 1;
  bool injectDynamicObstacles = true;
  std::size_t replanEverySteps = 35;
  std::size_t obstacleBatchSize = 10;
};

struct PlannerSpec {
  std::string name;
  std::function<std::unique_ptr<Planner>()> factory;
};

class BenchmarkRunner {
public:
  explicit BenchmarkRunner(std::vector<PlannerSpec> planners)
      : planners_(std::move(planners)) {}

  std::vector<BenchmarkMetrics> run(const BenchmarkConfig &config) const {
    std::vector<BenchmarkMetrics> results;
    for (int size : config.sizes) {
      for (const PlannerSpec &spec : planners_) {
        results.push_back(runSingle(size, spec, config));
      }
    }
    return results;
  }

  static void writeCsv(const std::vector<BenchmarkMetrics> &results,
                       const std::string &filePath) {
    std::ofstream out(filePath);
    if (!out) {
      throw std::runtime_error("Failed to open CSV file: " + filePath);
    }
        out << "planner,width,height,initial_plan_ms,total_replan_ms,total_wall_ms,"
          "memory_high_water_kb,path_length,path_cost,path_efficiency,turns,"
          "expansions,replan_count,injected_obstacles,reached_goal\n";
    out << std::fixed << std::setprecision(3);
    for (const BenchmarkMetrics &result : results) {
      out << result.plannerName << ',' << result.width << ',' << result.height
          << ',' << result.initialPlanMs << ',' << result.totalReplanMs << ','
          << result.totalWallMs << ',' << result.memoryHighWaterKb << ','
         << result.pathLength << ',' << result.pathCost << ','
         << result.pathEfficiency << ',' << result.turns << ','
         << result.expansions << ',' << result.replanCount << ','
         << result.injectedObstacles << ',' << (result.reachedGoal ? 1 : 0)
         << '\n';
    }
  }

private:
  BenchmarkMetrics runSingle(int size, const PlannerSpec &spec,
                             const BenchmarkConfig &config) const {
    TerrainGenerationConfig terrainConfig;
    terrainConfig.width = size;
    terrainConfig.height = size;
    terrainConfig.seed =
        config.seed ^ static_cast<std::uint32_t>(size * 2654435761u) ^
        static_cast<std::uint32_t>(std::hash<std::string>{}(spec.name));
    PhobosTerrainGenerator generator(terrainConfig);
    auto map = std::make_shared<GridMap>(generator.generate());

    const Cell start{0, 0};
    const Cell goal{static_cast<std::uint16_t>(size - 1),
            static_cast<std::uint16_t>(size - 1)};
    map->setCell(start, 1.0, false);
    map->setCell(goal, 1.0, false);

    auto planner = spec.factory();
    if (!planner) {
      throw std::runtime_error("Planner factory returned null for: " +
                               spec.name);
    }

    planner->initialize(map, start, goal);

    const auto wallBegin = std::chrono::steady_clock::now();
    std::vector<Cell> path = planner->computePath();
    Cell rover = start;
    std::size_t pathIndex = 0;
    std::size_t injectedObstacles = 0;
    std::mt19937 rng(
        config.seed ^ static_cast<std::uint32_t>(size * 17u) ^
        static_cast<std::uint32_t>(std::hash<std::string>{}(spec.name)));

    std::unique_ptr<visualization::Visualizer> visualizer;
    if (config.visualize) {
      std::filesystem::path outputPath(config.visualizationDirectory);
      outputPath /= spec.name;
      outputPath /= std::to_string(size) + "x" + std::to_string(size);
      visualizer = std::make_unique<visualization::PpmSequenceVisualizer>(
          outputPath, config.visualizationScale);
      visualizer->render(*map, path, rover, goal, 0);
    }

    std::size_t frameIndex = 1;
    std::size_t stepCount = 0;
    while (rover != goal) {
      if (path.empty()) {
        break;
      }
      if (pathIndex + 1 >= path.size()) {
        break;
      }

      rover = path[++pathIndex];
      ++stepCount;

      if (config.injectDynamicObstacles && config.replanEverySteps > 0 &&
          stepCount % config.replanEverySteps == 0 && rover != goal) {
        const std::vector<CellUpdate> updates = chooseDynamicUpdates(
            *map, path, pathIndex, config.obstacleBatchSize, rng, start, goal);
        if (!updates.empty()) {
          planner->applyUpdates(updates);
          injectedObstacles += updates.size();
          path = planner->replan(rover);
          pathIndex = 0;
        }
      }

      if (visualizer && (frameIndex % config.visualizationStride == 0)) {
        visualizer->render(*map, path, rover, goal, frameIndex);
      }
      ++frameIndex;

      if (path.size() == 1 && rover == goal) {
        break;
      }
      if (path.size() < 2 && rover != goal) {
        break;
      }
      if (pathIndex + 1 >= path.size() && rover != goal) {
        if (rover != goal) {
          path = planner->replan(rover);
          pathIndex = 0;
        }
      }
    }

    const auto wallEnd = std::chrono::steady_clock::now();

    BenchmarkMetrics metrics;
    metrics.plannerName = spec.name;
    metrics.width = size;
    metrics.height = size;
    metrics.initialPlanMs = planner->metrics().initialPlanMs;
    metrics.totalReplanMs = planner->metrics().totalReplanMs;
    metrics.totalWallMs =
        std::chrono::duration<double, std::milli>(wallEnd - wallBegin).count();
    metrics.memoryHighWaterKb = currentMemoryHighWaterKb();
    metrics.pathLength = map->estimatePathLength(path);
    metrics.pathCost = map->pathTraversalCost(path);
    metrics.pathEfficiency = pathEfficiency(*map, path);
    metrics.turns = countTurns(path);
    metrics.expansions = planner->metrics().expansions;
    metrics.replanCount = planner->metrics().replanCount;
    metrics.injectedObstacles = injectedObstacles;
    metrics.reachedGoal = (rover == goal);
    return metrics;
  }

  static std::size_t countTurns(const std::vector<Cell> &path) {
    if (path.size() < 3) {
      return 0;
    }

    std::size_t turns = 0;
    int previousDx = path[1].x - path[0].x;
    int previousDy = path[1].y - path[0].y;
    for (std::size_t i = 2; i < path.size(); ++i) {
      const int currentDx = path[i].x - path[i - 1].x;
      const int currentDy = path[i].y - path[i - 1].y;
      if (currentDx != previousDx || currentDy != previousDy) {
        ++turns;
      }
      previousDx = currentDx;
      previousDy = currentDy;
    }
    return turns;
  }

  static double straightLineDistance(const std::vector<Cell> &path) {
    if (path.size() < 2) {
      return 0.0;
    }
    const Cell &start = path.front();
    const Cell &goal = path.back();
    const double dx = static_cast<double>(goal.x - start.x);
    const double dy = static_cast<double>(goal.y - start.y);
    return std::sqrt(dx * dx + dy * dy);
  }

  static double pathEfficiency(const GridMap &map, const std::vector<Cell> &path) {
    const double directDistance = straightLineDistance(path);
    if (directDistance <= 0.0) {
      return 0.0;
    }
    return map.estimatePathLength(path) / directDistance;
  }

  static std::vector<CellUpdate>
  chooseDynamicUpdates(const GridMap &map, const std::vector<Cell> &path,
                       std::size_t pathIndex, std::size_t batchSize,
                       std::mt19937 &rng, Cell start, Cell goal) {
    std::vector<CellUpdate> updates;
    std::uniform_int_distribution<int> radiusDist(-3, 3);

    for (std::size_t i = 1; i <= batchSize; ++i) {
      Cell candidate;
      if (pathIndex + i < path.size()) {
        candidate = path[pathIndex + i];
      } else {
        const Cell pivot = path.empty() ? start : path[pathIndex];
        candidate = Cell{static_cast<std::uint16_t>(static_cast<int>(pivot.x) +
                                                     radiusDist(rng)),
                         static_cast<std::uint16_t>(static_cast<int>(pivot.y) +
                                                     radiusDist(rng))};
      }
      if (!map.inBounds(candidate) || candidate == start || candidate == goal ||
          map.isBlocked(candidate)) {
        continue;
      }
      updates.push_back(CellUpdate{candidate, true, 1.0, false});
    }

    return updates;
  }

#ifdef __linux__
  static double currentMemoryHighWaterKb() {
    struct rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
      return 0.0;
    }
    return static_cast<double>(usage.ru_maxrss);
  }
#else
  static double currentMemoryHighWaterKb() { return 0.0; }
#endif

  std::vector<PlannerSpec> planners_;
};

inline std::vector<PlannerSpec> defaultPlanners() {
  return {
      PlannerSpec{"field_dstar",
                  [] { return std::make_unique<FieldDStarPlanner>(); }},
      PlannerSpec{"dstar_lite",
                  [] { return std::make_unique<DStarLitePlanner>(); }},
  };
}

} // namespace path_planning
