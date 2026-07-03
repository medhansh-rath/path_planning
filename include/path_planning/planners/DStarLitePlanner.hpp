#pragma once

#include "path_planning/Planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace path_planning {

class DStarLitePlanner final : public Planner {
public:
  std::string name() const override { return "dstar_lite"; }

  void initialize(std::shared_ptr<GridMap> map, Cell start,
                  Cell goal) override {
    map_ = std::move(map);
    start_ = start;
    lastStart_ = start;
    goal_ = goal;
    metrics_ = {};
    km_ = 0.0;
    ensureStorage();
    std::fill(g_.begin(), g_.end(), infinity());
    std::fill(rhs_.begin(), rhs_.end(), infinity());
    clearOpen();
    rhs_[index(goal_)] = 0.0;
    push(goal_);
  }

  std::vector<Cell> computePath() override {
    const auto begin = std::chrono::steady_clock::now();
    computeShortestPath();
    const std::vector<Cell> path = extractPath();
    const auto end = std::chrono::steady_clock::now();
    metrics_.initialPlanMs =
        std::chrono::duration<double, std::milli>(end - begin).count();
    return path;
  }

  std::vector<Cell> replan(Cell newStart) override {
    const auto begin = std::chrono::steady_clock::now();
    km_ += heuristic(lastStart_, newStart);
    lastStart_ = newStart;
    start_ = newStart;
    computeShortestPath();
    const std::vector<Cell> path = extractPath();
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
      if (!map_->inBounds(update.cell)) {
        continue;
      }
      map_->setBlocked(update.cell, update.blocked);
      if (update.hasTraversalCost) {
        map_->setTraversalCost(update.cell, update.traversalCost);
      }
      updateVertex(update.cell);
      for (const Cell &neighbor : map_->neighbors(update.cell)) {
        updateVertex(neighbor);
      }
    }
  }

  const PathMetrics &metrics() const override { return metrics_; }

private:
  struct Key {
    double first = 0.0;
    double second = 0.0;
  };

  struct QueueNode {
    Key key;
    Cell cell;
  };

  struct QueueCompare {
    bool operator()(const QueueNode &lhs, const QueueNode &rhs) const {
      if (lhs.key.first != rhs.key.first) {
        return lhs.key.first > rhs.key.first;
      }
      return lhs.key.second > rhs.key.second;
    }
  };

  double heuristic(Cell lhs, Cell rhs) const {
    const double dx = static_cast<double>(lhs.x - rhs.x);
    const double dy = static_cast<double>(lhs.y - rhs.y);
    return std::sqrt(dx * dx + dy * dy);
  }

  Key calculateKey(Cell cell) const {
    const double best = std::min(g_[index(cell)], rhs_[index(cell)]);
    return Key{best + heuristic(start_, cell) + km_, best};
  }

  static bool lessKey(const Key &lhs, const Key &rhs) {
    if (lhs.first != rhs.first) {
      return lhs.first < rhs.first;
    }
    return lhs.second < rhs.second;
  }

  static bool equalKey(const Key &lhs, const Key &rhs) {
    return std::abs(lhs.first - rhs.first) < 1e-9 &&
           std::abs(lhs.second - rhs.second) < 1e-9;
  }

  void ensureStorage() {
    if (!map_) {
      return;
    }
    const std::size_t size = static_cast<std::size_t>(map_->width()) *
                             static_cast<std::size_t>(map_->height());
    g_.assign(size, infinity());
    rhs_.assign(size, infinity());
  }

  std::size_t index(Cell cell) const { return map_->index(cell); }

  void clearOpen() {
    while (!open_.empty()) {
      open_.pop();
    }
  }

  void push(Cell cell) { open_.push(QueueNode{calculateKey(cell), cell}); }

  double g(Cell cell) const { return g_[index(cell)]; }
  double rhs(Cell cell) const { return rhs_[index(cell)]; }
  double &g(Cell cell) { return g_[index(cell)]; }
  double &rhs(Cell cell) { return rhs_[index(cell)]; }

  void updateVertex(Cell cell) {
    if (cell != goal_) {
      double best = infinity();
      for (const Cell &neighbor : map_->neighbors(cell)) {
        const double candidate = map_->moveCost(cell, neighbor) + g(neighbor);
        if (candidate < best) {
          best = candidate;
        }
      }
      rhs(cell) = best;
    }
    if (!equalKeyKey(cell)) {
      push(cell);
    }
  }

  bool equalKeyKey(Cell cell) const {
    const double gValue = g(cell);
    const double rhsValue = rhs(cell);
    return std::abs(gValue - rhsValue) < 1e-9;
  }

  void computeShortestPath() {
    if (!map_) {
      return;
    }

    while (!open_.empty()) {
      const Key topKey = open_.top().key;
      const Key startKey = calculateKey(start_);
      if (!lessKey(topKey, startKey) &&
          std::abs(rhs(start_) - g(start_)) < 1e-9) {
        break;
      }

      const QueueNode current = open_.top();
      open_.pop();
      const Key currentKey = calculateKey(current.cell);
      if (!equalKey(current.key, currentKey)) {
        continue;
      }
      if (std::abs(g(current.cell) - rhs(current.cell)) < 1e-9) {
        continue;
      }

      ++metrics_.expansions;
      if (g(current.cell) > rhs(current.cell)) {
        g(current.cell) = rhs(current.cell);
        for (const Cell &neighbor : map_->neighbors(current.cell)) {
          updateVertex(neighbor);
        }
      } else {
        g(current.cell) = infinity();
        updateVertex(current.cell);
        for (const Cell &neighbor : map_->neighbors(current.cell)) {
          updateVertex(neighbor);
        }
      }
    }
  }

  std::vector<Cell> extractPath() const {
    if (!map_ || map_->isBlocked(start_) || map_->isBlocked(goal_)) {
      return {};
    }
    if (!std::isfinite(g(start_)) && start_ != goal_) {
      return {};
    }

    std::vector<Cell> path;
    path.push_back(start_);
    Cell current = start_;
    const std::size_t guardLimit = static_cast<std::size_t>(map_->width()) *
                                   static_cast<std::size_t>(map_->height());

    for (std::size_t guard = 0; guard < guardLimit && current != goal_;
         ++guard) {
      double bestScore = infinity();
      Cell bestNeighbor = current;
      for (const Cell &neighbor : map_->neighbors(current)) {
        const double candidate =
            map_->moveCost(current, neighbor) + g(neighbor);
        if (candidate < bestScore) {
          bestScore = candidate;
          bestNeighbor = neighbor;
        }
      }
      if (bestNeighbor == current || !std::isfinite(bestScore)) {
        return {};
      }
      current = bestNeighbor;
      path.push_back(current);
    }

    if (path.empty() || path.back() != goal_) {
      return {};
    }
    return path;
  }

  static double infinity() { return std::numeric_limits<double>::infinity(); }

  std::shared_ptr<GridMap> map_;
  Cell start_;
  Cell lastStart_;
  Cell goal_;
  double km_ = 0.0;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open_;
  PathMetrics metrics_;
};

} // namespace path_planning
