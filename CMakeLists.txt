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
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

# bullet double precision
# TODO
#add_definitions( -DBT_USE_DOUBLE_PRECISION)
#set( BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")

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

## --------------------------- local files ---------------------------
include_directories(include)

# Benchmark
set(Benchmark_HDRS
        include/interface/UserHandle.hpp
        )

# BulletSim
set(BulletSim_SRCS
        include/bulletSim/World.cpp
        include/bulletSim/World_RG.cpp
        include/bulletSim/object/SingleBodyObject.cpp
        include/bulletSim/object/Box.cpp
        include/bulletSim/object/CheckerBoard.cpp
        include/bulletSim/object/Sphere.cpp
        include/bulletSim/object/Object.cpp
        )

set(BulletSim_HDRS
        include/bulletSim/World.hpp
        include/bulletSim/World_RG.hpp
        include/bulletSim/object/SingleBodyObject.hpp
        include/bulletSim/object/Box.hpp
        include/bulletSim/object/CheckerBoard.hpp
        include/bulletSim/object/Sphere.hpp
        include/bulletSim/object/Object.hpp
        )

add_library(BulletSim ${BulletSim_SRCS} ${BulletSim_HDRS})

# ODESim
set(ODESim_SRCS
        include/odeSim/World.cpp
        include/odeSim/World_RG.cpp
        include/odeSim/object/SingleBodyObject.cpp
        include/odeSim/object/Box.cpp
        include/odeSim/object/CheckerBoard.cpp
        include/odeSim/object/Sphere.cpp
        )

set(ODESim_HDRS
        include/odeSim/World.hpp
        include/odeSim/World_RG.hpp
        include/odeSim/object/SingleBodyObject.hpp
        include/odeSim/object/Box.hpp
        include/odeSim/object/CheckerBoard.hpp
        include/odeSim/object/Sphere.hpp
        )

add_library(ODESim ${ODESim_SRCS} ${ODESim_HDRS})

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

## --------------------------- links ---------------------------
# BulletSim
set(BulletSimLink Bullet3Common BulletDynamics BulletCollision LinearMath BulletSim ${RAI_GRAPHICS_OPENGL_LINK})

# ODESim
set(ODESimLink ode ODESim ${RAI_GRAPHICS_OPENGL_LINK})

## --------------------------- executable ---------------------------
# framework tests
add_executable(raisimTest test/raisimTest.cpp)
target_link_libraries(raisimTest ${RAI_SIM_LINK})

add_executable(bulletTest test/bulletTest.cpp)
target_link_libraries(bulletTest BulletCollision BulletDynamics LinearMath)

add_executable(odeTest test/odeTest.cpp)
target_link_libraries(odeTest ode)

# benchmark tests
# stack
add_executable(boxStackRaiSim benchmark/stackRaiSim.cpp)
target_link_libraries(boxStackRaiSim ${RAI_SIM_LINK})

add_executable(boxStackBulletSim benchmark/stackBulletSim.cpp)
target_link_libraries(boxStackBulletSim ${BulletSimLink})

add_executable(boxStackODESim benchmark/stackODESim.cpp)
target_link_libraries(boxStackODESim ${ODESimLink})

# rolling
add_executable(rollingRaiSim benchmark/rollingRaiSim.cpp)
target_link_libraries(rollingRaiSim ${RAI_SIM_LINK})


# anymal
add_executable(anymalRaiSim benchmark/anymalRaiSim.cpp)
target_link_libraries(anymalRaiSim ${RAI_SIM_LINK})