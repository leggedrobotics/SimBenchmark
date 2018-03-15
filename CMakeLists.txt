cmake_minimum_required(VERSION 2.9)
project(benchmark)

## =========================== COMPILER ===========================
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
#set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)

## =========================== CMAKE MODE ===========================
# bullet debug
# TODO bullet debug check (in release mode)
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# raiCommons debug
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DRAIDEBUG)
else ()
    remove_definitions(-DRAIDEBUG)
endif()

# options
option(BENCHMARK_RAISIM OFF)
option(BENCHMARK_BULLET OFF)
option(BENCHMARK_ODE OFF)
option(BENCHMARK_MUJOCO OFF)

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

# raisim (optional)
if(BENCHMARK_RAISIM)
    message(STATUS "Benchmark raiSim option is on")

    # set raiSim backend
    add_definitions(-DCOLLISION_BACKEND_ODE)        # ODE
#    add_definitions(-DCOLLISION_BACKEND_BULLET)     # BULLET

    find_package(raiSim CONFIG REQUIRED)
    include_directories(${RAI_SIM_INCLUDE_DIR})
endif()

# bullet (optional)
if(BENCHMARK_BULLET)
    message(STATUS "Benchmark Bullet option is on")

    # bullet double precision
    add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

    find_package(Bullet CONFIG REQUIRED)
    include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
    link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
endif()

# ode (optional)
if(BENCHMARK_ODE)
    message(STATUS "Benchmark ODE option is on")

    # ode double precision
    add_definitions(-DODE_DOUBLE_PRECISION)

    find_package(ode CONFIG REQUIRED)
    include_directories(${ODE_INCLUDE_DIRS})
endif()

# mujoco (optional)
if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark mujoco option is on")

    include_directories(mjpro150/include)
    # mujoco shared libraries
    find_library(glew NAMES glew PATHS mjpro150/bin)
    find_library(glewegl NAMES glewegl PATHS mjpro150/bin)
    find_library(glewosmesa NAMES glewosmesa PATHS mjpro150/bin)
    find_library(glfw NAMES glfw libglfw.so.3 PATHS mjpro150/bin)
    find_library(mujoco150 NAMES mujoco150 PATHS mjpro150/bin)
    find_library(mujoco150nogl NAMES mujoco150nogl PATHS mjpro150/bin)
    # mujoco keyfile
    file(COPY mjpro150/mjkey.txt
            DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

## =========================== FILES ===========================
include_directories(src)

# Benchmark
set(Benchmark_SRCS
        src/base/UserHandle.hpp
        src/base/World_RG.hpp
        src/base/World_RG.cpp
        src/base/SingleBodyObject.hpp
        src/base/SingleBodyObject.cpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})

add_executable(yamlTest test/yamlTest.cpp)
target_link_libraries(yamlTest ${YAML_CPP_LIBRARIES})

# raiSim
if(BENCHMARK_RAISIM AND raiSim_FOUND)
    message("raiSim installation was found in local path")
endif()

# BulletSim
if(BENCHMARK_BULLET AND Bullet_FOUND)
    message("Bullet installation was found in local path")

    set(BulletSim_SRCS
            src/bulletSim/World.hpp
            src/bulletSim/World.cpp
            src/bulletSim/World_RG.hpp
            src/bulletSim/World_RG.cpp
            src/bulletSim/object/SingleBodyObject.hpp
            src/bulletSim/object/SingleBodyObject.cpp
            src/bulletSim/object/Box.hpp
            src/bulletSim/object/Box.cpp
            src/bulletSim/object/CheckerBoard.hpp
            src/bulletSim/object/CheckerBoard.cpp
            src/bulletSim/object/Sphere.hpp
            src/bulletSim/object/Sphere.cpp
            src/bulletSim/object/Capsule.hpp
            src/bulletSim/object/Capsule.cpp
            )

    add_library(BulletSim ${BulletSim_SRCS})

    ## --------------------------- links ---------------------------
    set(BulletSimLink BulletDynamics Bullet3Common BulletCollision LinearMath
            BulletSim BenchmarkLib ${RAI_COMMON_LINK} ${RAI_GRAPHICS_OPENGL_LINK})
endif()

# ODESim
if(BENCHMARK_ODE AND ode_FOUND)
    message("ODE installation was found in local path")

    set(ODESim_SRCS
            src/odeSim/World.hpp
            src/odeSim/World.cpp
            src/odeSim/World_RG.hpp
            src/odeSim/World_RG.cpp
            src/odeSim/object/SingleBodyObject.hpp
            src/odeSim/object/SingleBodyObject.cpp
            src/odeSim/object/Box.hpp
            src/odeSim/object/Box.cpp
            src/odeSim/object/CheckerBoard.hpp
            src/odeSim/object/CheckerBoard.cpp
            src/odeSim/object/Sphere.cpp
            src/odeSim/object/Sphere.hpp
            src/odeSim/object/Capsule.hpp
            src/odeSim/object/Capsule.cpp
            )

    add_library(ODESim ${ODESim_SRCS})

    ## --------------------------- link ---------------------------
    set(ODESimLink ode ODESim BenchmarkLib ${RAI_COMMON_LINK} ${RAI_GRAPHICS_OPENGL_LINK})
endif()

# MuJoCoSim
if(BENCHMARK_MUJOCO)
    set(MuJoCoSim_SRCS
            src/mujocoSim/World.cpp
            src/mujocoSim/World.hpp
            src/mujocoSim/World_RG.cpp
            src/mujocoSim/World_RG.hpp
            src/mujocoSim/object/SingleBodyObject.cpp
            src/mujocoSim/object/SingleBodyObject.hpp
            src/mujocoSim/object/Sphere.cpp
            src/mujocoSim/object/Sphere.hpp
            src/mujocoSim/object/Box.cpp
            src/mujocoSim/object/Box.hpp
            src/mujocoSim/object/Capsule.cpp
            src/mujocoSim/object/Capsule.hpp
            src/mujocoSim/object/CheckerBoard.cpp
            src/mujocoSim/object/CheckerBoard.hpp
            )

    add_library(MuJoCoSim ${MuJoCoSim_SRCS})

    ## --------------------------- links ---------------------------
    set(MuJoCoSimLink ${glew} ${glfw} ${mujoco150} ${mujoco150nogl}
            GL GLU glut MuJoCoSim BenchmarkLib ${RAI_COMMON_LINK} ${RAI_GRAPHICS_OPENGL_LINK})

    # mujoco simulation
    add_executable(mujocoSimulate mjpro150/sample/simulate.cpp)
    target_link_libraries(mujocoSimulate ${MuJoCoSimLink})

endif()

## =========================== EXECUTABLES ===========================
add_subdirectory(test)
add_subdirectory(demo)
add_subdirectory(benchmark)