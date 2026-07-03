#pragma once

#include "path_planning/GridMap.hpp"

#include <memory>
#include <string>
#include <vector>

namespace path_planning {

class Planner {
public:
  virtual ~Planner() = default;

  virtual std::string name() const = 0;
  virtual void initialize(std::shared_ptr<GridMap> map, Cell start,
                          Cell goal) = 0;
  virtual std::vector<Cell> computePath() = 0;
  virtual std::vector<Cell> replan(Cell newStart) = 0;
  virtual void applyUpdates(const std::vector<CellUpdate> &updates) = 0;
  virtual const PathMetrics &metrics() const = 0;
};

} // namespace path_planning
