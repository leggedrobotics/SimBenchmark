cmake_minimum_required(VERSION 2.9)
project(benchmark)

## =========================== COMPILER ===========================
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
#set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)

## =========================== MODE ===========================
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

# raisim (optional)
if(raiSim)
    message("benchmark raiSim")
    find_package(raiSim CONFIG)
    include_directories(${RAI_SIM_INCLUDE_DIR})
endif()

# bullet (optional)
if(BulletSim)
    message("benchmark Bullet")

    # bullet double precision
    add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

    find_package(Bullet CONFIG)
    include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
    link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
endif()

# ode (optional)
if(odeSim)
    message("benchmark ODE")
    find_package(ode CONFIG)
    include_directories(${ODE_INCLUDE_DIRS})
endif()

# mujoco (optional)
if(mujocoSim)
    message("benchmark mujoco")

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

# raiSim
if(raiSim AND raiSim_FOUND)
    message("raiSim installation was found in local path")

    add_executable(raisimTest test/raisimTest.cpp)
    target_link_libraries(raisimTest ${RAI_SIM_LINK})
endif()

# BulletSim
if(BulletSim AND Bullet_FOUND)
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

    ## --------------------------- tests ---------------------------
    add_executable(bulletTest test/bulletTest.cpp)
    target_link_libraries(bulletTest BulletCollision BulletDynamics LinearMath)
endif()

# ODESim
if(odeSim AND ode_FOUND)
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

    ## --------------------------- tests ---------------------------
    add_executable(odeTest test/odeTest.cpp)
    target_link_libraries(odeTest ode)
endif()

# MuJoCoSim
if(mujocoSim)
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

    ## --------------------------- tests ---------------------------
    add_executable(mujocoTest test/mujocoTest.cpp)
    target_link_libraries(mujocoTest ${MuJoCoSimLink})

endif()

## --------------------------- export files ---------------------------
file(COPY benchmark/shell/rolling.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY benchmark/shell/bounce.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

## =========================== DEMOS ===========================
if(raiSim AND raiSim_FOUND)
    # stack
    add_executable(stackRaiSim demo/stackRaiSim.cpp)
    target_link_libraries(stackRaiSim ${RAI_SIM_LINK})

    # anymal
    add_executable(anymalRaiSim benchmark/anymalRaiSim.cpp)
    target_link_libraries(anymalRaiSim ${RAI_SIM_LINK})

    # collision anymal
    add_executable(collisionAnymalRaiSim benchmark/collisionAnymalRaiSim.cpp)
    target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

    # free drop anymal
    add_executable(freeDropAnymalRaiSim benchmark/freeDropAnymalRaiSim.cpp)
    target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})
endif()

if(BulletSim AND Bullet_FOUND)
    # stack
    add_executable(stackBulletSim demo/stackBulletSim.cpp)
    target_link_libraries(stackBulletSim ${BulletSimLink})
endif()

if(odeSim AND ode_FOUND)
    # stack
    add_executable(stackODESim demo/stackODESim.cpp)
    target_link_libraries(stackODESim ${ODESimLink})

    # friction
    add_executable(frictionODESim demo/frictionODESim.cpp)
    target_link_libraries(frictionODESim ${ODESimLink})
endif()

## =========================== BENCHMARKS ===========================
if(raiSim AND raiSim_FOUND)
    # rolling
    add_executable(rollingRaiSim benchmark/rollingRaiSim.cpp)
    target_link_libraries(rollingRaiSim ${RAI_SIM_LINK})

    # bounce
    add_executable(bounceRaiSim benchmark/bounceRaiSim.cpp)
    target_link_libraries(bounceRaiSim ${RAI_SIM_LINK})

    # thousand
    add_executable(thousandRaiSim benchmark/thousandRaiSim.cpp)
    target_link_libraries(thousandRaiSim ${RAI_SIM_LINK})
endif()

if(BulletSim AND Bullet_FOUND)
    # rolling
    add_executable(rollingBulletSim benchmark/rollingBulletSim.cpp)
    target_link_libraries(rollingBulletSim ${BulletSimLink})

    # bounce
    add_executable(bounceBulletSim benchmark/bounceBulletSim.cpp)
    target_link_libraries(bounceBulletSim ${BulletSimLink})

    # thousand
    add_executable(thousandBulletSim benchmark/thousandBulletSim.cpp)
    target_link_libraries(thousandBulletSim ${BulletSimLink})
endif()

if(odeSim AND ode_FOUND)
    # rolling
    add_executable(rollingODESim benchmark/rollingODESim.cpp)
    target_link_libraries(rollingODESim ${ODESimLink})

    # bounce
    add_executable(bounceODESim benchmark/bounceODESim.cpp)
    target_link_libraries(bounceODESim ${ODESimLink})

    # thousand
    add_executable(thousandODESim benchmark/thousandODESim.cpp)
    target_link_libraries(thousandODESim ${ODESimLink})
endif()

if(mujocoSim)
    ## --------------------------- benchmark test ---------------------------
    # rolling
    add_executable(rollingMuJoCoSim benchmark/rollingMuJoCoSim.cpp)
    target_link_libraries(rollingMuJoCoSim ${MuJoCoSimLink})

    # bouncing
    add_executable(bounceMuJoCoSim benchmark/bounceMuJoCoSim.cpp)
    target_link_libraries(bounceMuJoCoSim ${MuJoCoSimLink})
endif()