########################################################################################################################
# dependencies

# bullet
externalproject_get_property(ex_bullet INSTALL_DIR)
set(bullet_include_dir "${INSTALL_DIR}/include")
set(bullet_lib_dir "${INSTALL_DIR}/lib")

find_library(BulletRobotics
        NO_DEFAULT_PATH
        NAMES       BulletRobotics
        PATHS       ${bullet_lib_dir})
find_library(BulletFileLoader
        NO_DEFAULT_PATH
        NAMES       BulletFileLoader
        PATHS       ${bullet_lib_dir})
find_library(BulletWorldImporter
        NO_DEFAULT_PATH
        NAMES       BulletWorldImporter
        PATHS       ${bullet_lib_dir})
find_library(BulletSoftBody
        NO_DEFAULT_PATH
        NAMES       BulletSoftBody
        PATHS       ${bullet_lib_dir})
find_library(BulletDynamics
        NO_DEFAULT_PATH
        NAMES       BulletDynamics
        PATHS       ${bullet_lib_dir})
find_library(BulletCollision
        NO_DEFAULT_PATH
        NAMES       BulletCollision
        PATHS       ${bullet_lib_dir})
find_library(BulletInverseDynamicsUtils
        NO_DEFAULT_PATH
        NAMES       BulletInverseDynamicsUtils
        PATHS       ${bullet_lib_dir})
find_library(BulletInverseDynamics
        NO_DEFAULT_PATH
        NAMES       BulletInverseDynamics
        PATHS       ${bullet_lib_dir})
find_library(LinearMath
        NO_DEFAULT_PATH
        NAMES       LinearMath
        PATHS       ${bullet_lib_dir})
find_library(Bullet3Common
        NO_DEFAULT_PATH
        NAMES       Bullet3Common
        PATHS       ${bullet_lib_dir})


if(NOT BulletRobotics
        OR NOT BulletFileLoader
        OR NOT BulletWorldImporter
        OR NOT BulletSoftBody
        OR NOT BulletDynamics
        OR NOT BulletCollision
        OR NOT BulletInverseDynamicsUtils
        OR NOT BulletInverseDynamics
        OR NOT LinearMath
        OR NOT Bullet3Common)
    message(WARNING "Cannot find bullet libs. Stop build bulletSim")
    return()
endif()


# raigraphics
externalproject_get_property(ex_raigraphics INSTALL_DIR)
find_package(raigraphics
        CONFIG
        NO_DEFAULT_PATH
        PATHS           "${INSTALL_DIR}"
        )


########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native -march=native")

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
message(STATUS "  bulletSim for Bullet Physics")

add_library(bulletSim
        STATIC
        ${bulletSimSrc})

add_dependencies(bulletSim
        ex_bullet
        ex_raigraphics
        commonSim
        )

target_include_directories(bulletSim
        PUBLIC
        "src"
        "${bullet_include_dir}"
        "${raigraphics_INCLUDE_DIRS}"
        )


target_link_libraries(bulletSim
        ${BulletRobotics}
        ${BulletFileLoader}
        ${BulletWorldImporter}
        ${BulletSoftBody}
        ${BulletDynamics}
        ${BulletCollision}
        ${BulletInverseDynamicsUtils}
        ${BulletInverseDynamics}
        ${LinearMath}
        ${Bullet3Common}
        commonSim
        ${CMAKE_DL_LIBS}
        )

########################################################################################################################
# sub-directories
if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)