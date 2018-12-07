########################################################################################################################
# dependencies
find_package(Bullet CONFIG REQUIRED)
find_package(raigraphics CONFIG REQUIRED)


########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)


########################################################################################################################
# source
set(bulletSimSrc
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


########################################################################################################################
# library
message(STATUS "  Build bulletSim")

add_library(bulletSim
        STATIC
        ${bulletSimSrc})
target_include_directories(bulletSim
        PUBLIC
        src
        ${raigraphics_INCLUDE_DIRS}
)
target_link_libraries(bulletSim
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
        commonSim
        ${CMAKE_DL_LIBS}
)

########################################################################################################################
# sub-directories
if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)