#pragma once

#include "path_planning/Planner.hpp"

#include <stdexcept>

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
    throw std::logic_error("Field D* is not implemented yet");
  }

  std::vector<Cell> replan(Cell /*newStart*/) override {
    throw std::logic_error("Field D* is not implemented yet");
  }

  void applyUpdates(const std::vector<CellUpdate> & /*updates*/) override {}

  const PathMetrics &metrics() const override { return metrics_; }

private:
  std::shared_ptr<GridMap> map_;
  Cell start_;
  Cell goal_;
  PathMetrics metrics_;
};

} // namespace path_planning
