#pragma once

#include "path_planning/visualization/Visualizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace path_planning::visualization {

class PpmSequenceVisualizer final : public Visualizer {
public:
  PpmSequenceVisualizer(std::filesystem::path outputDirectory, int scale = 1)
      : outputDirectory_(std::move(outputDirectory)),
        scale_(std::max(1, scale)) {
    std::filesystem::create_directories(outputDirectory_);
  }

  void render(const GridMap &map, const std::vector<Cell> &path, Cell rover,
              Cell goal, std::size_t frameIndex) override {
    const int imageWidth = map.width() * scale_;
    const int imageHeight = map.height() * scale_;
    std::vector<unsigned char> pixels(
        static_cast<std::size_t>(imageWidth) * imageHeight * 3, 0);

    std::unordered_set<Cell, CellHash> pathCells(path.begin(), path.end());

    const double maxCost = std::max(1.0, map.maxTraversalCost());
    for (int y = 0; y < map.height(); ++y) {
      for (int x = 0; x < map.width(); ++x) {
        const Cell cell{x, y};
        const auto color = colorForCell(map, cell, pathCells.count(cell) != 0,
                                        cell == rover, cell == goal, maxCost);
        for (int sy = 0; sy < scale_; ++sy) {
          for (int sx = 0; sx < scale_; ++sx) {
            const int pixelX = x * scale_ + sx;
            const int pixelY = y * scale_ + sy;
            const std::size_t pixelIndex =
                static_cast<std::size_t>(pixelY) * imageWidth + pixelX;
            pixels[pixelIndex * 3 + 0] = color[0];
            pixels[pixelIndex * 3 + 1] = color[1];
            pixels[pixelIndex * 3 + 2] = color[2];
          }
        }
      }
    }

    const std::filesystem::path framePath =
        outputDirectory_ / frameName(frameIndex);
    std::ofstream out(framePath, std::ios::binary);
    if (!out) {
      throw std::runtime_error("Failed to open frame file: " +
                               framePath.string());
    }
    out << "P6\n" << imageWidth << ' ' << imageHeight << "\n255\n";
    out.write(reinterpret_cast<const char *>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
  }

private:
  static std::array<unsigned char, 3> colorForCell(const GridMap &map,
                                                   Cell cell, bool onPath,
                                                   bool rover, bool goal,
                                                   double maxCost) {
    if (map.isBlocked(cell)) {
      return {28, 28, 32};
    }
    if (goal) {
      return {250, 220, 60};
    }
    if (rover) {
      return {240, 70, 60};
    }
    if (onPath) {
      return {60, 150, 245};
    }

    const double normalized =
        std::clamp(map.traversalCost(cell) / maxCost, 0.0, 1.0);
    const double red = 34.0 + 190.0 * normalized;
    const double green = 56.0 + 140.0 * (1.0 - normalized);
    const double blue = 36.0 + 44.0 * (1.0 - normalized);
    return {static_cast<unsigned char>(red), static_cast<unsigned char>(green),
            static_cast<unsigned char>(blue)};
  }

  static std::string frameName(std::size_t frameIndex) {
    std::ostringstream oss;
    oss << "frame_" << std::setw(6) << std::setfill('0') << frameIndex
        << ".ppm";
    return oss.str();
  }

  std::filesystem::path outputDirectory_;
  int scale_ = 1;
};

} // namespace path_planning::visualization
