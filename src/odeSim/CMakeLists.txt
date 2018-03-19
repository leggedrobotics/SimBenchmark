if(BENCHMARK_ODE AND ode_FOUND)
    message(STATUS "ODE installation was found in local path")

    set(ODESim_SRCS
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
            object/Sphere.cpp
            object/Sphere.hpp
            object/Capsule.hpp
            object/Capsule.cpp
            )

    add_library(ODESimLib ${ODESim_SRCS})
endif()