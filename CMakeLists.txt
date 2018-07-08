cmake_minimum_required(VERSION 2.9)
project(benchmark)

## =========================== COMPILER ===========================
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
#set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)

## =========================== CMAKE MODE ===========================
# raiCommons debug
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DRAIDEBUG)
else ()
    remove_definitions(-DRAIDEBUG)
endif()

# options
option(BENCHMARK_RAISIM "benchmark raiSim" ON)
option(BENCHMARK_BULLET_MULTIBODY "benchmark Bullet Multibody(bullet3 API)" ON)
option(BENCHMARK_ODE "benchmark ODE" ON)
option(BENCHMARK_MUJOCO "benchmark MUJOCO" ON)
option(BENCHMARK_DART "benchmark DART" ON)
option(BUILD_DEMO "build demos" OFF)
option(BUILD_TEST "build tests" OFF)
option(BUILD_BENCHMARK "build benchmark tests" ON)
#option(BUILD_UTIL "build utils" ON)

## =========================== DEPENDENCIES ===========================
set(BENCHMARK_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

# global libs
# eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# boost
find_package(Boost COMPONENTS system filesystem program_options REQUIRED )
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

## yaml-cpp
#find_package(YAML-CPP REQUIRED)
#include_directories(${YAML_CPP_INCLUDE_DIR})

# local libs
add_subdirectory(lib)
include_directories(${BENCHMARK_LIB_DIR}/raiCommon/include)         # raiCommon
include_directories(${BENCHMARK_LIB_DIR}/raiGraphics/include)       # raiGraphics
include_directories(${BENCHMARK_LIB_DIR}/yaml-cpp/include)          # yaml-cpp

## =========================== LINK ===========================
set(BENCHMARK_LINK
        CommonLib
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${RAI_COMMON_LINK}
        ${RAI_GRAPHICS_OPENGL_LINK}
        )

## =========================== SIMS ===========================
include_directories(sim)
add_subdirectory(sim)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)