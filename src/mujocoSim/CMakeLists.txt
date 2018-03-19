if(BENCHMARK_MUJOCO)
    set(MuJoCoSim_SRCS
            World.cpp
            World.hpp
            World_RG.cpp
            World_RG.hpp
            object/SingleBodyObject.cpp
            object/SingleBodyObject.hpp
            object/Sphere.cpp
            object/Sphere.hpp
            object/Box.cpp
            object/Box.hpp
            object/Capsule.cpp
            object/Capsule.hpp
            object/CheckerBoard.cpp
            object/CheckerBoard.hpp
            )

    add_library(MuJoCoSimLib ${MuJoCoSim_SRCS})
endif()
