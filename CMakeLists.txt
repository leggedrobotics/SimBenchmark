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

# BulletSim
set(BulletSim_SRCS
        src/bulletSim/World.cpp
        src/bulletSim/World_RG.cpp
        src/bulletSim/SingleBodyObject.cpp
        src/bulletSim/Box.cpp
        src/bulletSim/CheckerBoard.cpp
        )

set(BulletSim_HDRS
        include/bulletSim/World.hpp
        include/bulletSim/World_RG.hpp
        include/bulletSim/object/SingleBodyObject.hpp
        include/bulletSim/object/Box.hpp
        include/bulletSim/object/CheckerBoard.hpp
        )

add_library(BulletSim ${BulletSim_SRCS} ${BulletSim_HDRS})

# ODESim
set(ODESim_SRCS
        src/odeSim/World.cpp
        src/odeSim/World_RG.cpp
        src/odeSim/SingleBodyObject.cpp
        src/odeSim/Box.cpp
        src/odeSim/CheckerBoard.cpp
        )

set(ODESim_HDRS
        include/odeSim/World.hpp
        include/odeSim/World_RG.hpp
        include/odeSim/object/SingleBodyObject.hpp
        include/odeSim/object/Box.hpp
        include/bulletSim/object/CheckerBoard.hpp
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
set(BulletSimLink BulletCollision BulletDynamics LinearMath BulletSim ${RAI_GRAPHICS_OPENGL_LINK})

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
add_executable(boxStackRaiSim benchmark/boxStackRaiSim.cpp)
target_link_libraries(boxStackRaiSim ${RAI_SIM_LINK})

add_executable(boxStackBulletSim benchmark/boxStackBulletSim.cpp)
target_link_libraries(boxStackBulletSim ${BulletSimLink})

add_executable(boxStackODESim benchmark/boxStackODESim.cpp)
target_link_libraries(boxStackODESim ${ODESimLink})