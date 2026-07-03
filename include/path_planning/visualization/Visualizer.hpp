#pragma once

#include "path_planning/GridMap.hpp"

#include <string>
#include <vector>

namespace path_planning::visualization {

class Visualizer {
public:
  virtual ~Visualizer() = default;

  virtual void render(const GridMap &map, const std::vector<Cell> &path,
                      Cell rover, Cell goal, std::size_t frameIndex) = 0;
};

} // namespace path_planning::visualization
