cmake_minimum_required(VERSION 3.9)
project(benchmark)

## --------------------------- compiler ---------------------------
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wl,--no-as-needed -O3 -mtune=native -march=native")

## --------------------------- mode ---------------------------
# bullet debug
# TODO bullet debug check (in release mode)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

# raiCommons debug
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DRAIDEBUG)
    set(CMAKE_CXX_FLAGS "-Wl,--no-as-needed -O0 -mtune=native -march=native")
else ()
    remove_definitions(-DRAIDEBUG)
    set(CMAKE_CXX_FLAGS "-Wl,--no-as-needed -O3 -mtune=native -march=native")
endif()

## --------------------------- link directories ---------------------------
link_directories(/usr/local/lib)
link_directories(/usr/lib)
link_directories(/usr/lib/x86_64-linux-gnu/)

## --------------------------- dependencies ---------------------------
# raiCommons
find_package(raiCommon CONFIG REQUIRED)
include_directories(${RAI_COMMON_INCLUDE_DIR})

# raiGraphics
find_package(raiGraphicsOpengl CONFIG REQUIRED)
include_directories(${RAI_GRAPHICS_OPENGL_INCLUDE_DIRS})

# raisim
find_package(raiSim CONFIG REQUIRED)
include_directories(${RAI_SIM_INCLUDE_DIR})

# bullet
find_package(Bullet CONFIG REQUIRED)
include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})

# ode
find_package(ode CONFIG REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# mujoco
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

## --------------------------- local files ---------------------------
include_directories(include)

# Benchmark
set(Benchmark_SRCS
        include/base/UserHandle.hpp
        include/base/World_RG.hpp
        include/base/World_RG.cpp
        include/base/SingleBodyObject.hpp
        include/base/SingleBodyObject.cpp
        )

add_library(BenchmakrLib ${Benchmark_SRCS})

# BulletSim
set(BulletSim_SRCS
        include/bulletSim/World.hpp
        include/bulletSim/World.cpp
        include/bulletSim/World_RG.hpp
        include/bulletSim/World_RG.cpp
        include/bulletSim/object/SingleBodyObject.hpp
        include/bulletSim/object/SingleBodyObject.cpp
        include/bulletSim/object/Box.hpp
        include/bulletSim/object/Box.cpp
        include/bulletSim/object/CheckerBoard.hpp
        include/bulletSim/object/CheckerBoard.cpp
        include/bulletSim/object/Sphere.hpp
        include/bulletSim/object/Sphere.cpp
        include/bulletSim/object/Capsule.hpp
        include/bulletSim/object/Capsule.cpp
        )

add_library(BulletSim ${BulletSim_SRCS})

# ODESim
set(ODESim_SRCS
        include/odeSim/World.hpp
        include/odeSim/World.cpp
        include/odeSim/World_RG.hpp
        include/odeSim/World_RG.cpp
        include/odeSim/object/SingleBodyObject.hpp
        include/odeSim/object/SingleBodyObject.cpp
        include/odeSim/object/Box.hpp
        include/odeSim/object/Box.cpp
        include/odeSim/object/CheckerBoard.hpp
        include/odeSim/object/CheckerBoard.cpp
        include/odeSim/object/Sphere.cpp
        include/odeSim/object/Sphere.hpp
        include/odeSim/object/Capsule.hpp
        include/odeSim/object/Capsule.cpp
        )

add_library(ODESim ${ODESim_SRCS})

# MuJoCoSim
set(MuJoCoSim_SRCS
        include/mujocoSim/World.hpp
        include/mujocoSim/World.cpp
        include/mujocoSim/object/SingleBodyObject.hpp
        include/mujocoSim/object/SingleBodyObject.cpp
        )

add_library(MuJoCoSim ${MuJoCoSim_SRCS})

## --------------------------- links ---------------------------
# BulletSim
set(BulletSimLink BulletDynamics Bullet3Common BulletCollision LinearMath
        BulletSim BenchmakrLib ${RAI_GRAPHICS_OPENGL_LINK})

# ODESim
set(ODESimLink ode ODESim BenchmakrLib ${RAI_GRAPHICS_OPENGL_LINK})

# MuJoCoSim
set(MuJoCoSimLink ${glew} ${glfw} ${mujoco150} ${mujoco150nogl} GL GLU glut MuJoCoSim ${RAI_GRAPHICS_OPENGL_LINK})

## --------------------------- executable ---------------------------
# framework tests
add_executable(raisimTest test/raisimTest.cpp)
target_link_libraries(raisimTest ${RAI_SIM_LINK})

add_executable(bulletTest test/bulletTest.cpp)
target_link_libraries(bulletTest BulletCollision BulletDynamics LinearMath)

add_executable(odeTest test/odeTest.cpp)
target_link_libraries(odeTest ode)

add_executable(mujocoTest test/mujocoTest.cpp)
target_link_libraries(mujocoTest ${MuJoCoSimLink})

# mujoco simulation
add_executable(mujocoSimulate mjpro150/sample/simulate.cpp)
target_link_libraries(mujocoSimulate ${MuJoCoSimLink})

# benchmark tests
# stack
add_executable(stackRaiSim benchmark/stackRaiSim.cpp)
target_link_libraries(stackRaiSim ${RAI_SIM_LINK})

add_executable(stackBulletSim benchmark/stackBulletSim.cpp)
target_link_libraries(stackBulletSim ${BulletSimLink})

add_executable(stackODESim benchmark/stackODESim.cpp)
target_link_libraries(stackODESim ${ODESimLink})

# rolling
add_executable(rollingRaiSim benchmark/rollingRaiSim.cpp)
target_link_libraries(rollingRaiSim ${RAI_SIM_LINK})

add_executable(rollingBulletSim benchmark/rollingBulletSim.cpp)
target_link_libraries(rollingBulletSim ${BulletSimLink})

add_executable(rollingODESim benchmark/rollingODESim.cpp)
target_link_libraries(rollingODESim ${ODESimLink})

# bouncing
add_executable(bounceRaiSim benchmark/bounceRaiSim.cpp)
target_link_libraries(bounceRaiSim ${RAI_SIM_LINK})

add_executable(bounceBulletSim benchmark/bounceBulletSim.cpp)
target_link_libraries(bounceBulletSim ${BulletSimLink})

add_executable(bounceODESim benchmark/bounceODESim.cpp)
target_link_libraries(bounceODESim ${ODESimLink})

add_executable(bounceMuJoCoSim benchmark/bounceMuJoCoSim.cpp)
target_link_libraries(bounceMuJoCoSim ${MuJoCoSimLink})

# thousand
add_executable(thousandRaiSim benchmark/thousandRaiSim.cpp)
target_link_libraries(thousandRaiSim ${RAI_SIM_LINK})

add_executable(thousandBulletSim benchmark/thousandBulletSim.cpp)
target_link_libraries(thousandBulletSim ${BulletSimLink})

add_executable(thousandODESim benchmark/thousandODESim.cpp)
target_link_libraries(thousandODESim ${ODESimLink})

# anymal
add_executable(anymalRaiSim benchmark/anymalRaiSim.cpp)
target_link_libraries(anymalRaiSim ${RAI_SIM_LINK})

# collision anymal
add_executable(collisionAnymalRaiSim benchmark/collisionAnymalRaiSim.cpp)
target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

# free drop anymal
add_executable(freeDropAnymalRaiSim benchmark/freeDropAnymalRaiSim.cpp)
target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})

# friction
add_executable(frictionODESim benchmark/frictionODESim.cpp)
target_link_libraries(frictionODESim ${ODESimLink})
