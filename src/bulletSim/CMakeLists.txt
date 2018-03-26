set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

if(BENCHMARK_BULLET AND Bullet_FOUND)
    message(STATUS "Bullet installation was found in local path")

    set(BulletSim_SRCS
            World.hpp
            World.cpp
            World_RG.hpp
            World_RG.cpp
            UserHandle.hpp
            object/Object.hpp
            object/Object.cpp
            object/SingleBodyObject.hpp
            object/SingleBodyObject.cpp
            object/Box.hpp
            object/Box.cpp
            object/CheckerBoard.hpp
            object/CheckerBoard.cpp
            object/Sphere.hpp
            object/Sphere.cpp
            object/Capsule.hpp
            object/Capsule.cpp
            object/ArticulatedSystem/ArticulatedSystem.hpp
            object/ArticulatedSystem/ArticulatedSystem.cpp
            object/ArticulatedSystem/URDF/MyMultiBodyCreator.h
            object/ArticulatedSystem/URDF/MyMultiBodyCreator.cpp
            object/ArticulatedSystem/URDF/URDFToBullet.h
            object/ArticulatedSystem/URDF/URDFToBullet.cpp
#            object/ArticulatedSystem/URDF/BulletUrdfImporter.h
#            object/ArticulatedSystem/URDF/BulletUrdfImporter.cpp
            )

    add_library(BulletSimLib ${BulletSim_SRCS})
    target_link_libraries(BulletSimLib
            BulletDynamics
            Bullet3Common
            BulletCollision
            LinearMath
            BulletRobotics
            )
endif()