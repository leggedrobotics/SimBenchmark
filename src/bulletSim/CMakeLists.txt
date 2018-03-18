if(BENCHMARK_BULLET AND Bullet_FOUND)
    message(STATUS "Bullet installation was found in local path")

    set(BulletSim_SRCS
            World.hpp
            World.cpp
            World_RG.hpp
            World_RG.cpp
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
            )

    add_library(BulletSim ${BulletSim_SRCS})

    ## --------------------------- links ---------------------------
    set(BulletSimLink
            BulletDynamics
            Bullet3Common
            BulletCollision
            LinearMath
            BulletSim
            ${BENCHMARK_LINK}
            ${RAI_COMMON_LINK}
            ${RAI_GRAPHICS_OPENGL_LINK}
            )
endif()