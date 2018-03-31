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
option(BENCHMARK_ODE "benchmark ODE" OFF)
option(BENCHMARK_MUJOCO "benchmark MUJOCO" ON)
option(BENCHMARK_DART "benchmark DART" OFF)

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
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(YAML-CPP REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# boost
find_package( Boost COMPONENTS system filesystem program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
link_directories(${Boost_LIBRARY_DIRS})

# raisim (optional)
if(BENCHMARK_RAISIM)
    message(STATUS "Benchmark raiSim")

    # set raiSim backend
    add_definitions(-DCOLLISION_BACKEND_ODE)        # ODE
    #    add_definitions(-DCOLLISION_BACKEND_BULLET)     # BULLET

    # raiSim
    find_package(raiSim CONFIG REQUIRED)
    include_directories(${RAI_SIM_INCLUDE_DIR})
endif()

# bullet (optional)
if(BENCHMARK_BULLET)
    message(STATUS "Benchmark Bullet")

    # bullet double precision
    add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

    find_package(Bullet CONFIG REQUIRED)
    include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
    link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
endif()

# ode (optional)
if(BENCHMARK_ODE)
    message(STATUS "Benchmark ODE")

    # ode double precision
    add_definitions(-DODE_DOUBLE_PRECISION)

    find_package(ode CONFIG REQUIRED)
    include_directories(${ODE_INCLUDE_DIRS})
endif()

# mujoco (optional)
if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark MUJOCO")

    include_directories(lib/mjpro150/include)
    # mujoco shared libraries
    find_library(glew NAMES glew
            PATHS lib/mjpro150/bin)
    find_library(glewegl NAMES glewegl
            PATHS lib/mjpro150/bin)
    find_library(glewosmesa NAMES glewosmesa
            PATHS lib/mjpro150/bin)
    find_library(glfw NAMES glfw libglfw.so.3
            PATHS lib/mjpro150/bin)
    find_library(mujoco150 NAMES mujoco150
            PATHS lib/mjpro150/bin)
    find_library(mujoco150nogl NAMES mujoco150nogl
            PATHS lib/mjpro150/bin)
    # mujoco keyfile
    file(COPY lib/mjpro150/mjkey.txt
            DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

# dart (optional)
if(BENCHMARK_DART)
    message(STATUS "Benchmark DART")

    find_package(DART CONFIG REQUIRED)
    include_directories(${DART_INCLUDE_DIRS})
endif()


## =========================== LINK ===========================
set(BENCHMARK_LINK
        BenchmarkLib
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${RAI_COMMON_LINK}
        ${RAI_GRAPHICS_OPENGL_LINK}
        )

if(BENCHMARK_BULLET AND Bullet_FOUND)
    set(BulletSimLink
            BulletSimLib
            ${BENCHMARK_LINK}
            )
endif()

if(BENCHMARK_ODE AND ode_FOUND)
    set(ODESimLink
            ODESimLib
            ${BENCHMARK_LINK}
            )
endif()

if(BENCHMARK_MUJOCO)
    set(MuJoCoSimLink
            MuJoCoSimLib
            ${BENCHMARK_LINK}
            )
endif()

if(BENCHMARK_DART)
    set(DartSimLink
            DartSimLib
            ${BENCHMARK_LINK}
            )
endif()

## =========================== FILES ===========================
include_directories(src)
add_subdirectory(src)

# raiSim
if(BENCHMARK_RAISIM AND raiSim_FOUND)
    message(STATUS "raiSim installation was found in local path")
endif()

add_subdirectory(test)
add_subdirectory(demo)
add_subdirectory(benchmark)