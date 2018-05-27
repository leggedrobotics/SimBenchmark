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
option(BENCHMARK_BULLET "benchmark Bullet" ON)
option(BENCHMARK_BULLET_MULTIBODY "benchmark Bullet Multibody" ON)
option(BENCHMARK_ODE "benchmark ODE" ON)
option(BENCHMARK_MUJOCO "benchmark MUJOCO" ON)
option(BENCHMARK_DART "benchmark DART" ON)
option(BUILD_DEMO "build demos" ON)
option(BUILD_TEST "build tests" ON)
#option(BUILD_UTIL "build utils" ON)

## =========================== LINK DIRECTORIES ===========================
link_directories(/usr/local/lib)
link_directories(/usr/lib)
link_directories(/usr/lib/x86_64-linux-gnu/)

## =========================== DEPENDENCIES ===========================
# raiCommons
find_package(raiCommon CONFIG REQUIRED)
include_directories(${RAI_COMMON_INCLUDE_DIR})

# raiGraphics
find_package(raiGraphicsOpengl CONFIG REQUIRED)
include_directories(${RAI_GRAPHICS_OPENGL_INCLUDE_DIRS})

# eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(YAML-CPP REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# boost
find_package( Boost COMPONENTS system filesystem program_options REQUIRED )
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

## =========================== LINK ===========================
set(BENCHMARK_LINK
        CommonLib
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${RAI_COMMON_LINK}
        ${RAI_GRAPHICS_OPENGL_LINK}
        )

## =========================== FILES ===========================
include_directories(sim)
add_subdirectory(sim)
add_subdirectory(benchmark)

#file(INSTALL ${CMAKE_SOURCE_DIR}/res
#        DESTINATION ${CMAKE_BINARY_DIR})

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)