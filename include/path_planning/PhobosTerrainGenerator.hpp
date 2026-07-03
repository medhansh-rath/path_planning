#pragma once

#include "path_planning/GridMap.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace path_planning {

struct TerrainGenerationConfig {
  int width = 100;
  int height = 100;
  std::uint32_t seed = 42;
  double slopeStrength = 2.0;
  double noiseStrength = 0.9;
  double obstacleDensity = 0.025;
  double obstacleClusterChance = 0.12;
};

class PhobosTerrainGenerator {
public:
  explicit PhobosTerrainGenerator(TerrainGenerationConfig config = {})
      : config_(config) {}

  GridMap generate() const {
    GridMap map(config_.width, config_.height);
    std::mt19937 rng(config_.seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    std::uniform_real_distribution<double> slopeDist(-1.0, 1.0);
    std::uniform_real_distribution<double> phaseDist(0.0, 6.283185307179586);

    const double slopeX = slopeDist(rng);
    const double slopeY = slopeDist(rng);
    const double phaseA = phaseDist(rng);
    const double phaseB = phaseDist(rng);
    const double phaseC = phaseDist(rng);
    const double phaseD = phaseDist(rng);

    std::vector<double> elevation(
        static_cast<std::size_t>(config_.width) * config_.height, 0.0);

    for (int y = 0; y < config_.height; ++y) {
      for (int x = 0; x < config_.width; ++x) {
        const double nx =
            static_cast<double>(x) / std::max(1, config_.width - 1);
        const double ny =
            static_cast<double>(y) / std::max(1, config_.height - 1);
        const double smoothWave =
            0.35 *
                std::sin((nx * 8.0 + ny * 2.0) * 6.283185307179586 + phaseA) +
            0.25 *
                std::cos((nx * 3.0 - ny * 7.0) * 6.283185307179586 + phaseB) +
            0.18 *
                std::sin((nx * 13.0 + ny * 11.0) * 6.283185307179586 + phaseC) +
            0.14 * std::cos((nx * 5.0 + ny * 9.0) * 6.283185307179586 + phaseD);
        const double gradient =
            config_.slopeStrength * (0.7 * slopeX * nx + 0.9 * slopeY * ny);
        elevation[static_cast<std::size_t>(y) * config_.width + x] =
            smoothWave + gradient;
      }
    }

    auto sampleElevation = [&](int x, int y) {
      x = std::clamp(x, 0, config_.width - 1);
      y = std::clamp(y, 0, config_.height - 1);
      return elevation[static_cast<std::size_t>(y) * config_.width + x];
    };

    for (int y = 0; y < config_.height; ++y) {
      for (int x = 0; x < config_.width; ++x) {
        const double center = sampleElevation(x, y);
        const double dx =
            0.5 * (sampleElevation(x + 1, y) - sampleElevation(x - 1, y));
        const double dy =
            0.5 * (sampleElevation(x, y + 1) - sampleElevation(x, y - 1));
        const double slopeMagnitude = std::sqrt(dx * dx + dy * dy);
        const double localNoise =
            std::abs(std::sin(phaseA + 0.17 * x + 0.11 * y)) +
            std::abs(std::cos(phaseB + 0.07 * x - 0.13 * y));
        const double traversalCost =
            1.0 + config_.noiseStrength *
                      (0.65 * slopeMagnitude + 0.15 * std::abs(center) +
                       0.12 * localNoise);
        const bool obstacle = isObstacleCell(x, y, uniform01(rng));
        map.setCell(Cell{x, y}, traversalCost, obstacle);
      }
    }

    return map;
  }

private:
  bool isObstacleCell(int x, int y, double randomValue) const {
    const double cellNoise = hashedNoise(x, y, config_.seed ^ 0x9e3779b9u);
    const double clusterNoise =
        hashedNoise(x / 2, y / 2, config_.seed ^ 0x85ebca6bu);
    const double density =
        config_.obstacleDensity +
        (clusterNoise < config_.obstacleClusterChance ? 0.06 : 0.0);
    return randomValue < density * (0.55 + 0.45 * cellNoise);
  }

  static double hashedNoise(int x, int y, std::uint32_t seed) {
    std::uint64_t value =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) * 73856093ULL;
    value ^=
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) * 19349663ULL;
    value ^= static_cast<std::uint64_t>(seed) * 83492791ULL;
    value ^= value >> 13;
    value *= 1274126177ULL;
    value ^= value >> 16;
    return static_cast<double>(value & 0xFFFFFFULL) /
           static_cast<double>(0x1000000ULL);
  }

  TerrainGenerationConfig config_;
};

} // namespace path_planning
