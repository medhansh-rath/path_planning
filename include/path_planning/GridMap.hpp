#pragma once

#include "path_planning/Types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <stdexcept>
#include <vector>

namespace path_planning {

class GridMap {
public:
  GridMap() = default;

  GridMap(int width, int height)
      : width_(width), height_(height),
        traversalCost_(static_cast<std::size_t>(width) * height, 1.0),
        blocked_(static_cast<std::size_t>(width) * height, 0) {
    if (width <= 0 || height <= 0) {
      throw std::invalid_argument("GridMap dimensions must be positive");
    }
  }

  int width() const { return width_; }
  int height() const { return height_; }

  bool inBounds(Cell cell) const {
    return static_cast<int>(cell.x) < width_ &&
           static_cast<int>(cell.y) < height_;
  }

  std::size_t index(Cell cell) const {
    return static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(cell.x);
  }

  Cell cellFromIndex(std::size_t index) const {
    return Cell{static_cast<std::uint16_t>(index % static_cast<std::size_t>(width_)),
                static_cast<std::uint16_t>(index / static_cast<std::size_t>(width_))};
  }

  bool isBlocked(Cell cell) const { return blocked_.at(index(cell)) != 0; }

  void setBlocked(Cell cell, bool blocked) {
    blocked_.at(index(cell)) = blocked ? 1 : 0;
  }

  double traversalCost(Cell cell) const {
    return traversalCost_.at(index(cell));
  }

  void setTraversalCost(Cell cell, double cost) {
    traversalCost_.at(index(cell)) = std::max(0.0001, cost);
  }

  void setCell(Cell cell, double cost, bool blocked) {
    setTraversalCost(cell, cost);
    setBlocked(cell, blocked);
  }

  double moveCost(Cell from, Cell to) const {
    if (!inBounds(from) || !inBounds(to) || isBlocked(from) || isBlocked(to)) {
      return std::numeric_limits<double>::infinity();
    }
    const double dx = static_cast<double>(to.x - from.x);
    const double dy = static_cast<double>(to.y - from.y);
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double averageCost = 0.5 * (traversalCost(from) + traversalCost(to));
    return distance * averageCost;
  }

  std::vector<Cell> neighbors(Cell cell) const {
    static constexpr std::array<std::pair<int, int>, 8> offsets{{
        std::pair<int, int>{-1, -1},
        std::pair<int, int>{0, -1},
        std::pair<int, int>{1, -1},
        std::pair<int, int>{-1, 0},
        std::pair<int, int>{1, 0},
        std::pair<int, int>{-1, 1},
        std::pair<int, int>{0, 1},
        std::pair<int, int>{1, 1},
    }};

    std::vector<Cell> result;
    result.reserve(8);
    for (const std::pair<int, int> &offset : offsets) {
      const int nextX = static_cast<int>(cell.x) + offset.first;
      const int nextY = static_cast<int>(cell.y) + offset.second;
      if (nextX < 0 || nextY < 0) {
        continue;
      }
      Cell next{static_cast<std::uint16_t>(nextX), static_cast<std::uint16_t>(nextY)};
      if (!inBounds(next) || isBlocked(next)) {
        continue;
      }
      if (std::abs(offset.first) == 1 && std::abs(offset.second) == 1) {
        const int adjacentAX = static_cast<int>(cell.x) + offset.first;
        const int adjacentAY = static_cast<int>(cell.y);
        const int adjacentBX = static_cast<int>(cell.x);
        const int adjacentBY = static_cast<int>(cell.y) + offset.second;
        if (adjacentAX < 0 || adjacentAY < 0 || adjacentBX < 0 || adjacentBY < 0) {
          continue;
        }
        const Cell adjacentA{static_cast<std::uint16_t>(adjacentAX),
                             static_cast<std::uint16_t>(adjacentAY)};
        const Cell adjacentB{static_cast<std::uint16_t>(adjacentBX),
                             static_cast<std::uint16_t>(adjacentBY)};
        if (!inBounds(adjacentA) || !inBounds(adjacentB) ||
            isBlocked(adjacentA) || isBlocked(adjacentB)) {
          continue;
        }
      }
      result.push_back(next);
    }
    return result;
  }

  double estimatePathLength(const std::vector<Cell> &path) const {
    if (path.size() < 2) {
      return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double dx = static_cast<double>(path[i].x - path[i - 1].x);
      const double dy = static_cast<double>(path[i].y - path[i - 1].y);
      length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
  }

  double pathTraversalCost(const std::vector<Cell> &path) const {
    if (path.size() < 2) {
      return 0.0;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      cost += moveCost(path[i - 1], path[i]);
    }
    return cost;
  }

  double maxTraversalCost() const {
    return *std::max_element(traversalCost_.begin(), traversalCost_.end());
  }

private:
  int width_ = 0;
  int height_ = 0;
  std::vector<double> traversalCost_;
  std::vector<std::uint8_t> blocked_;
};

} // namespace path_planning
