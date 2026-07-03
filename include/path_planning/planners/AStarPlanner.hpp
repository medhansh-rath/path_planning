#pragma once

#include "path_planning/Planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace path_planning {

class AStarPlanner final : public Planner {
public:
  std::string name() const override { return "astar"; }

  void initialize(std::shared_ptr<GridMap> map, Cell start,
                  Cell goal) override {
    map_ = std::move(map);
    start_ = start;
    goal_ = goal;
    metrics_ = {};
  }

  std::vector<Cell> computePath() override {
    const auto begin = std::chrono::steady_clock::now();
    std::vector<Cell> path = search(start_);
    const auto end = std::chrono::steady_clock::now();
    metrics_.initialPlanMs =
        std::chrono::duration<double, std::milli>(end - begin).count();
    return path;
  }

  std::vector<Cell> replan(Cell newStart) override {
    start_ = newStart;
    const auto begin = std::chrono::steady_clock::now();
    std::vector<Cell> path = search(start_);
    const auto end = std::chrono::steady_clock::now();
    metrics_.totalReplanMs +=
        std::chrono::duration<double, std::milli>(end - begin).count();
    ++metrics_.replanCount;
    return path;
  }

  void applyUpdates(const std::vector<CellUpdate> &updates) override {
    for (const CellUpdate &update : updates) {
      if (!map_ || !map_->inBounds(update.cell)) {
        continue;
      }
      map_->setBlocked(update.cell, update.blocked);
      if (update.hasTraversalCost) {
        map_->setTraversalCost(update.cell, update.traversalCost);
      }
    }
  }

  const PathMetrics &metrics() const override { return metrics_; }

private:
  struct QueueNode {
    double f = 0.0;
    double g = 0.0;
    Cell cell;
  };

  struct QueueCompare {
    bool operator()(const QueueNode &lhs, const QueueNode &rhs) const {
      if (lhs.f != rhs.f) {
        return lhs.f > rhs.f;
      }
      return lhs.g > rhs.g;
    }
  };

  double heuristic(Cell from) const {
    const double dx = static_cast<double>(goal_.x - from.x);
    const double dy = static_cast<double>(goal_.y - from.y);
    return std::sqrt(dx * dx + dy * dy);
  }

  std::vector<Cell> search(Cell start) {
    if (!map_ || !map_->inBounds(start) || !map_->inBounds(goal_)) {
      return {};
    }
    if (map_->isBlocked(start) || map_->isBlocked(goal_)) {
      return {};
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
    std::unordered_map<Cell, double, CellHash> gScore;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;
    gScore[start] = 0.0;
    open.push(QueueNode{heuristic(start), 0.0, start});

    while (!open.empty()) {
      const QueueNode current = open.top();
      open.pop();
      ++metrics_.expansions;

      auto currentScoreIt = gScore.find(current.cell);
      if (currentScoreIt == gScore.end() ||
          current.g != currentScoreIt->second) {
        continue;
      }
      if (current.cell == goal_) {
        return reconstructPath(cameFrom, current.cell);
      }

      for (const Cell &next : map_->neighbors(current.cell)) {
        const double tentativeG =
            currentScoreIt->second + map_->moveCost(current.cell, next);
        auto nextScoreIt = gScore.find(next);
        if (nextScoreIt == gScore.end() || tentativeG < nextScoreIt->second) {
          gScore[next] = tentativeG;
          cameFrom[next] = current.cell;
          open.push(QueueNode{tentativeG + heuristic(next), tentativeG, next});
        }
      }
    }

    return {};
  }

  std::vector<Cell>
  reconstructPath(const std::unordered_map<Cell, Cell, CellHash> &cameFrom,
                  Cell current) const {
    std::vector<Cell> path;
    path.push_back(current);
    while (current != start_) {
      const auto it = cameFrom.find(current);
      if (it == cameFrom.end()) {
        return {};
      }
      current = it->second;
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::shared_ptr<GridMap> map_;
  Cell start_;
  Cell goal_;
  PathMetrics metrics_;
};

} // namespace path_planning
