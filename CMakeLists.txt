cmake_minimum_required(VERSION 3.5)
project(simbenchmark)


########################################################################################################################
# ascii logo

message("")
message("===========================================================================")
message("   _____ ______  _______  _______   __________  ____  ______    ____  __ __")
message("  / ___//  _/  |/  / __ )/ ____/ | / / ____/ / / /  |/  /   |  / __ \\/ //_/")
message("  \\__ \\ / // /|_/ / __  / __/ /  |/ / /   / /_/ / /|_/ / /| | / /_/ / ,<   ")
message(" ___/ // // /  / / /_/ / /___/ /|  / /___/ __  / /  / / ___ |/ _, _/ /| |  ")
message("/____/___/_/  /_/_____/_____/_/ |_/\\____/_/ /_/_/  /_/_/  |_/_/ |_/_/ |_|  ")
message("                                                                           ")
message("===========================================================================")
message("")
message("This project is maintaied by Dongho Kang")
message("Contact kangd@ethz.ch for bug report")
message("")


########################################################################################################################
## compiler
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
#set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)


########################################################################################################################
# cmake mode

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DRAIDEBUG)
else ()
    remove_definitions(-DRAIDEBUG)
endif()


########################################################################################################################
# options
option(RAISIM "benchmark raiSim" OFF)
option(BULLET "benchmark Bullet Multibody(bullet3 API)" ON)
option(ODE "benchmark ODE" ON)
option(MUJOCO "benchmark MUJOCO" ON)
option(DART "benchmark DART" ON)

option(BUILD_DEMO "build demos" ON)
option(BUILD_TEST "build tests" ON)
option(BUILD_BENCHMARK "build benchmark tests" ON)


########################################################################################################################

message(STATUS "Benchmark the following physics engines:")
if(RAISIM)
    message(STATUS "  raiSim")
endif(RAISIM)

if(BULLET)
    message(STATUS "  Bullet")
endif(BULLET)

if(ODE)
    message(STATUS "  ODE")
endif(ODE)

if(MUJOCO)
    message(STATUS "  MUJOCO")
endif(MUJOCO)

if(DART)
    message(STATUS "  DART")
endif(DART)


########################################################################################################################
# dependencies

# global libs
# eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# boost
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# external projects
# this libraries will be installed in the build directory
add_subdirectory(lib)

########################################################################################################################
# macro variable

add_definitions(-DROOTPATH="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DRESOURCEPATH="${CMAKE_CURRENT_SOURCE_DIR}/res")


########################################################################################################################
# sims

include_directories(sim)
add_subdirectory(sim)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)