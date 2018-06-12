set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

# bullet package
include_directories(${BENCHMARK_LIB_DIR}/bullet/include)
link_directories(${BENCHMARK_LIB_DIR}/bullet)

# source
include_directories(src)

set(BtMbSimSrc
        src/BtMbWorld.hpp
        src/BtMbWorld.cpp
        src/BtMbSim.cpp
        src/BtMbSim.hpp
        src/BtMbUserHandle.hpp
        src/object/BtMbArticulatedSystem.cpp
        src/object/BtMbArticulatedSystem.hpp
        src/object/BtMbSingleBodyObject.cpp
        src/object/BtMbSingleBodyObject.hpp
        src/object/BtMbCheckerBoard.cpp
        src/object/BtMbCheckerBoard.hpp
        src/object/BtMbObject.cpp
        src/object/BtMbObject.hpp
        src/object/BtMbSphere.cpp
        src/object/BtMbSphere.hpp
        src/object/BtMbBox.cpp
        src/object/BtMbBox.hpp)

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

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)