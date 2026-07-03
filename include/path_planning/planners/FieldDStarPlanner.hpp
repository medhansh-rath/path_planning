#pragma once

#include "path_planning/Planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace path_planning {

class FieldDStarPlanner final : public Planner {
public:
  std::string name() const override { return "field_dstar"; }

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
    if (!map_) {
      return;
    }

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
    std::unordered_map<Cell, Cell, CellHash> parent;
    gScore[start] = 0.0;
    parent[start] = start;
    open.push(QueueNode{heuristic(start), 0.0, start});

    while (!open.empty()) {
      const QueueNode current = open.top();
      open.pop();
      auto currentScoreIt = gScore.find(current.cell);
      if (currentScoreIt == gScore.end() ||
          current.g != currentScoreIt->second) {
        continue;
      }

      ++metrics_.expansions;
      if (current.cell == goal_) {
        return densifyPath(reconstructWaypoints(parent, current.cell));
      }

      const Cell parentCell = parent.count(current.cell) != 0
                                  ? parent.at(current.cell)
                                  : current.cell;

      for (const Cell &neighbor : map_->neighbors(current.cell)) {
        Cell candidateParent = current.cell;
        if (parentCell != current.cell && lineOfSight(parentCell, neighbor)) {
          candidateParent = parentCell;
        }

        const double tentativeG =
            gScore[candidateParent] + segmentCost(candidateParent, neighbor);
        if (!std::isfinite(tentativeG)) {
          continue;
        }

        auto neighborScoreIt = gScore.find(neighbor);
        if (neighborScoreIt == gScore.end() ||
            tentativeG < neighborScoreIt->second) {
          gScore[neighbor] = tentativeG;
          parent[neighbor] = candidateParent;
          open.push(QueueNode{tentativeG + heuristic(neighbor), tentativeG,
                              neighbor});
        }
      }
    }

    return {};
  }

  std::vector<Cell>
  reconstructWaypoints(const std::unordered_map<Cell, Cell, CellHash> &parent,
                       Cell current) const {
    std::vector<Cell> waypoints;
    waypoints.push_back(current);
    while (current != start_) {
      const auto it = parent.find(current);
      if (it == parent.end()) {
        return {};
      }
      current = it->second;
      waypoints.push_back(current);
    }
    std::reverse(waypoints.begin(), waypoints.end());
    return waypoints;
  }

  std::vector<Cell> densifyPath(const std::vector<Cell> &waypoints) const {
    if (waypoints.empty()) {
      return {};
    }

    std::vector<Cell> path;
    path.push_back(waypoints.front());
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const std::vector<Cell> segment =
          traceLine(waypoints[i - 1], waypoints[i]);
      for (std::size_t j = 1; j < segment.size(); ++j) {
        path.push_back(segment[j]);
      }
    }
    return path;
  }

  std::vector<Cell> traceLine(Cell from, Cell to) const {
    std::vector<Cell> cells;
    cells.push_back(from);

    int x0 = from.x;
    int y0 = from.y;
    const int x1 = to.x;
    const int y1 = to.y;
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;

    while (x0 != x1 || y0 != y1) {
      const int doubleError = 2 * error;
      if (doubleError >= dy) {
        error += dy;
        x0 += sx;
      }
      if (doubleError <= dx) {
        error += dx;
        y0 += sy;
      }
      cells.push_back(Cell{x0, y0});
    }

    return cells;
  }

  bool lineOfSight(Cell from, Cell to) const {
    const std::vector<Cell> cells = traceLine(from, to);
    if (cells.size() < 2) {
      return true;
    }

    for (std::size_t i = 1; i < cells.size(); ++i) {
      const Cell previous = cells[i - 1];
      const Cell current = cells[i];
      if (!map_->inBounds(current) || map_->isBlocked(current)) {
        return false;
      }
      if (!isStepTraversable(previous, current)) {
        return false;
      }
    }

    return true;
  }

  bool isStepTraversable(Cell from, Cell to) const {
    if (!map_->inBounds(from) || !map_->inBounds(to) || map_->isBlocked(from) ||
        map_->isBlocked(to)) {
      return false;
    }

    const int dx = std::abs(to.x - from.x);
    const int dy = std::abs(to.y - from.y);
    if (dx == 1 && dy == 1) {
      const Cell horizontal{to.x, from.y};
      const Cell vertical{from.x, to.y};
      if (!map_->inBounds(horizontal) || !map_->inBounds(vertical) ||
          map_->isBlocked(horizontal) || map_->isBlocked(vertical)) {
        return false;
      }
    }

    return true;
  }

  double segmentCost(Cell from, Cell to) const {
    if (!map_ || !map_->inBounds(from) || !map_->inBounds(to) ||
        map_->isBlocked(from) || map_->isBlocked(to)) {
      return std::numeric_limits<double>::infinity();
    }

    const std::vector<Cell> cells = traceLine(from, to);
    if (cells.size() < 2) {
      return 0.0;
    }

    double cost = 0.0;
    for (std::size_t i = 1; i < cells.size(); ++i) {
      if (!isStepTraversable(cells[i - 1], cells[i])) {
        return std::numeric_limits<double>::infinity();
      }
      cost += map_->moveCost(cells[i - 1], cells[i]);
    }
    return cost;
  }

  std::shared_ptr<GridMap> map_;
  Cell start_;
  Cell goal_;
  PathMetrics metrics_;
};

} // namespace path_planning
