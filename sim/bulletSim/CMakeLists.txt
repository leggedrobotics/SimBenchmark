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

set(BULLET_SIM_SRC
        src/World.hpp
        src/World.cpp
        src/World_RG.hpp
        src/World_RG.cpp
        src/UserHandle.hpp
        src/object/Object.hpp
        src/object/Object.cpp
        src/object/SingleBodyObject.hpp
        src/object/SingleBodyObject.cpp
        src/object/Box.hpp
        src/object/Box.cpp
        src/object/CheckerBoard.hpp
        src/object/CheckerBoard.cpp
        src/object/Sphere.hpp
        src/object/Sphere.cpp
        src/object/Capsule.hpp
        src/object/Capsule.cpp
        src/object/Cylinder.hpp
        src/object/Cylinder.cpp
        src/object/ArticulatedSystem/ArticulatedSystem.hpp
        src/object/ArticulatedSystem/ArticulatedSystem.cpp
        src/object/ArticulatedSystem/URDF/MyMultiBodyCreator.h
        src/object/ArticulatedSystem/URDF/MyMultiBodyCreator.cpp
        src/object/ArticulatedSystem/URDF/URDFToBullet.h
        src/object/ArticulatedSystem/URDF/URDFToBullet.cpp
        #            src/object/ArticulatedSystem/URDF/BulletUrdfImporter.h
        #            src/object/ArticulatedSystem/URDF/BulletUrdfImporter.cpp
        )

# library
add_library(BULLET_SIM_LIB ${BULLET_SIM_SRC})
target_link_libraries(BULLET_SIM_LIB
        BulletDynamics
        Bullet3Common
        BulletCollision
        LinearMath
        BulletRobotics
        )

# link
set(BULLET_SIM_LINK
        BULLET_SIM_LIB
        ${BENCHMARK_LINK}
        )

# sub-directories
add_subdirectory(test)
add_subdirectory(demo)