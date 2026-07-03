#include "path_planning/Benchmark.hpp"

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<int> parseSizes(const std::string &text) {
  std::vector<int> sizes;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      sizes.push_back(std::stoi(token));
    }
  }
  return sizes;
}

std::vector<std::string> parsePlannerNames(const std::string &text) {
  std::vector<std::string> names;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      names.push_back(token);
    }
  }
  return names;
}

bool hasFlag(const std::vector<std::string> &args, const std::string &flag) {
  return std::find(args.begin(), args.end(), flag) != args.end();
}

std::string valueForPrefix(const std::vector<std::string> &args,
                           const std::string &prefix,
                           const std::string &fallback) {
  for (const std::string &arg : args) {
    if (arg.rfind(prefix, 0) == 0) {
      return arg.substr(prefix.size());
    }
  }
  return fallback;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<std::string> args;
  args.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  path_planning::BenchmarkConfig config;
  config.sizes = parseSizes(
      valueForPrefix(args, "--sizes=", "100,200,300,400,500,600,700,800"));
  config.seed = static_cast<std::uint32_t>(
      std::stoul(valueForPrefix(args, "--seed=", "42")));
  config.visualize = hasFlag(args, "--visualize");
  config.visualizationDirectory =
      valueForPrefix(args, "--output=", config.visualizationDirectory);
  config.visualizationScale =
      std::max(1, std::stoi(valueForPrefix(args, "--scale=", "1")));
  config.visualizationStride = static_cast<std::size_t>(
      std::max(1, std::stoi(valueForPrefix(args, "--frame-stride=", "1"))));
  config.injectDynamicObstacles = !hasFlag(args, "--static");
  config.replanEverySteps = static_cast<std::size_t>(
      std::max(1, std::stoi(valueForPrefix(args, "--replan-every=", "35"))));
  config.obstacleBatchSize = static_cast<std::size_t>(
      std::max(1, std::stoi(valueForPrefix(args, "--obstacle-batch=", "10"))));

  std::vector<path_planning::PlannerSpec> planners =
      path_planning::defaultPlanners();
  const std::string plannerFilter = valueForPrefix(args, "--planners=", "");
  if (!plannerFilter.empty()) {
    const std::vector<std::string> requested = parsePlannerNames(plannerFilter);
    std::vector<path_planning::PlannerSpec> filtered;
    for (const auto &planner : planners) {
      if (std::find(requested.begin(), requested.end(), planner.name) !=
          requested.end()) {
        filtered.push_back(planner);
      }
    }
    planners = std::move(filtered);
  }

  if (planners.empty()) {
    std::cerr << "No planners selected.\n";
    return 1;
  }

  path_planning::BenchmarkRunner runner(std::move(planners));
  const auto results = runner.run(config);
  path_planning::BenchmarkRunner::writeCsv(results, "benchmark_results.csv");

  std::cout << "planner,width,height,initial_plan_ms,total_replan_ms,total_"
               "wall_ms,memory_high_water_kb,path_length,path_cost,path_"
               "efficiency,turns,expansions,replan_count,injected_obstacles,"
               "reached_goal\n";
  std::cout << std::fixed << std::setprecision(3);
  for (const auto &row : results) {
    std::cout << row.plannerName << ',' << row.width << ',' << row.height << ','
              << row.initialPlanMs << ',' << row.totalReplanMs << ','
              << row.totalWallMs << ',' << row.memoryHighWaterKb << ','
              << row.pathLength << ',' << row.pathCost << ','
              << row.pathEfficiency << ',' << row.turns << ','
              << row.expansions << ',' << row.replanCount << ','
              << row.injectedObstacles << ',' << (row.reachedGoal ? 1 : 0)
              << '\n';
  }

  return 0;
}
