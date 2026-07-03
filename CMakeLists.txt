cmake_minimum_required(VERSION 3.16)
project(path_planning_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(path_planning_benchmark
    src/main.cpp
)

target_include_directories(path_planning_benchmark PRIVATE include)

target_compile_options(path_planning_benchmark PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)
