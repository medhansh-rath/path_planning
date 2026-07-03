#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace path_planning {

struct Cell {
  std::uint16_t x = 0;
  std::uint16_t y = 0;
};

inline bool operator==(const Cell &lhs, const Cell &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool operator!=(const Cell &lhs, const Cell &rhs) {
  return !(lhs == rhs);
}

struct CellHash {
  std::size_t operator()(const Cell &cell) const noexcept {
    const std::size_t packed = (static_cast<std::size_t>(cell.x) << 16) |
                               static_cast<std::size_t>(cell.y);
    return packed ^ (packed >> 23) ^ (packed << 11);
  }
};

struct CellUpdate {
  Cell cell;
  bool blocked = false;
  double traversalCost = 1.0;
  bool hasTraversalCost = false;
};

struct PathMetrics {
  double initialPlanMs = 0.0;
  double totalReplanMs = 0.0;
  std::size_t expansions = 0;
  std::size_t replanCount = 0;
};

struct BenchmarkMetrics {
  std::string plannerName;
  int width = 0;
  int height = 0;
  double initialPlanMs = 0.0;
  double totalReplanMs = 0.0;
  double totalWallMs = 0.0;
  double memoryHighWaterKb = 0.0;
  double pathLength = 0.0;
  double pathCost = 0.0;
  double pathEfficiency = 0.0;
  std::size_t expansions = 0;
  std::size_t turns = 0;
  std::size_t replanCount = 0;
  std::size_t injectedObstacles = 0;
  bool reachedGoal = false;
};

} // namespace path_planning
