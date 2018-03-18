if(BENCHMARK_ODE AND ode_FOUND)
    message(STATUS "ODE installation was found in local path")

    set(ODESim_SRCS
            src/odeSim/World.hpp
            src/odeSim/World.cpp
            src/odeSim/World_RG.hpp
            src/odeSim/World_RG.cpp
            src/odeSim/object/SingleBodyObject.hpp
            src/odeSim/object/SingleBodyObject.cpp
            src/odeSim/object/Box.hpp
            src/odeSim/object/Box.cpp
            src/odeSim/object/CheckerBoard.hpp
            src/odeSim/object/CheckerBoard.cpp
            src/odeSim/object/Sphere.cpp
            src/odeSim/object/Sphere.hpp
            src/odeSim/object/Capsule.hpp
            src/odeSim/object/Capsule.cpp
            )

    add_library(ODESim ${ODESim_SRCS})

    ## --------------------------- link ---------------------------
    set(ODESimLink
            ode
            ODESim
            ${BENCHMARK_LINK}
            ${RAI_COMMON_LINK}
            ${RAI_GRAPHICS_OPENGL_LINK}
            )
endif()