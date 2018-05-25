set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

# bullet package
find_package(Bullet CONFIG REQUIRED)
include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})

# source
include_directories(src)

set(BtSimSrc
        src/BtWorld.hpp
        src/BtWorld.cpp
        src/BtSim.hpp
        src/BtSim.cpp
        src/UserHandle.hpp
        src/object/BtObject.hpp
        src/object/BtObject.cpp
        src/object/BtSingleBodyObject.hpp
        src/object/BtSingleBodyObject.cpp
        src/object/BtBox.hpp
        src/object/BtBox.cpp
        src/object/BtCheckerBoard.hpp
        src/object/BtCheckerBoard.cpp
        src/object/BtSphere.hpp
        src/object/BtSphere.cpp
        src/object/BtCapsule.hpp
        src/object/BtCapsule.cpp
        src/object/BtCylinder.hpp
        src/object/BtCylinder.cpp
        src/object/ArticulatedSystem/BtArticulatedSystem.hpp
        src/object/ArticulatedSystem/BtArticulatedSystem.cpp
        src/object/ArticulatedSystem/URDF/MyMultiBodyCreator.h
        src/object/ArticulatedSystem/URDF/MyMultiBodyCreator.cpp
        src/object/ArticulatedSystem/URDF/URDFToBullet.h
        src/object/ArticulatedSystem/URDF/URDFToBullet.cpp
        src/object/ArticulatedSystem/URDF/BulletUrdfImporter.h
        src/object/ArticulatedSystem/URDF/BulletUrdfImporter.cpp
        )

# library
add_library(BtSimLib ${BtSimSrc})
target_link_libraries(BtSimLib
        BulletDynamics
        Bullet3Common
        BulletCollision
        LinearMath
        BulletRobotics
        )

# link
set(BtSimLink
        BtSimLib
        ${BENCHMARK_LINK}
        )

# sub-directories
add_subdirectory(benchmark)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)