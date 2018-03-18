if(BENCHMARK_MUJOCO)
    set(MuJoCoSim_SRCS
            src/mujocoSim/World.cpp
            src/mujocoSim/World.hpp
            src/mujocoSim/World_RG.cpp
            src/mujocoSim/World_RG.hpp
            src/mujocoSim/object/SingleBodyObject.cpp
            src/mujocoSim/object/SingleBodyObject.hpp
            src/mujocoSim/object/Sphere.cpp
            src/mujocoSim/object/Sphere.hpp
            src/mujocoSim/object/Box.cpp
            src/mujocoSim/object/Box.hpp
            src/mujocoSim/object/Capsule.cpp
            src/mujocoSim/object/Capsule.hpp
            src/mujocoSim/object/CheckerBoard.cpp
            src/mujocoSim/object/CheckerBoard.hpp
            )

    add_library(MuJoCoSim ${MuJoCoSim_SRCS})

    ## --------------------------- links ---------------------------
    set(MuJoCoSimLink ${glew} ${glfw} ${mujoco150} ${mujoco150nogl}
            GL GLU glut MuJoCoSim ${BENCHMARK_LINK} ${RAI_COMMON_LINK} ${RAI_GRAPHICS_OPENGL_LINK})

endif()
