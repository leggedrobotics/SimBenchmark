cmake_minimum_required(VERSION 3.9)
project(benchmark)

set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-Wl,--no-as-needed -O3 -funroll-loops")

link_directories(/usr/local/lib)
link_directories(/usr/lib)
link_directories(/usr/lib/x86_64-linux-gnu/)

## --------------------------- local files ---------------------------
include_directories(include)

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

## --------------------------- executable ---------------------------
# demo basic
add_executable(raisimTest test/raisimTest.cpp)
target_link_libraries(raisimTest ${RAI_SIM_LINK})

add_executable(bulletTest test/bulletTest.cpp)
target_link_libraries(bulletTest BulletCollision BulletDynamics LinearMath)

add_executable(benchmark main.cpp)
target_link_libraries(benchmark BulletCollision BulletDynamics LinearMath)

add_executable(boxStackRaiSim benchmark/boxStackRaiSim.cpp)
target_link_libraries(boxStackRaiSim ${RAI_SIM_LINK})
