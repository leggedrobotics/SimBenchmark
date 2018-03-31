set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

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
    target_link_libraries(MuJoCoSimLib
            ${glew}
            ${glfw}
            ${mujoco150}
            ${mujoco150nogl}
            GL
            GLU
            glut)
endif()
