set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

# bullet package
find_package(Bullet CONFIG REQUIRED)
include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
message(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})

# source
include_directories(src)

set(BtMbSimSrc
        src/BtMbWorld.hpp
        src/BtMbWorld.cpp
        src/api/b3RobotSimulatorClientAPI_NoGUI.h
        src/api/b3RobotSimulatorClientAPI_NoGUI.cpp
        src/api/b3RobotSimulatorClientAPI.h
        src/api/b3RobotSimulatorClientAPI.cpp
        src/BtMbWorldRG.cpp
        src/BtMbWorldRG.hpp)

# library
add_library(BtMbSimLib ${BtMbSimSrc})
target_link_libraries(BtMbSimLib
        BulletRobotics
        BulletFileLoader
        BulletWorldImporter
        BulletSoftBody
        BulletDynamics
        BulletCollision
        BulletInverseDynamicsUtils
        BulletInverseDynamics
        LinearMath
        Bullet3Common
        )

# link
set(BtMbSimLink
        BtMbSimLib
        ${BENCHMARK_LINK}
        )

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)