set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

if(BENCHMARK_ODE AND ode_FOUND)
    message(STATUS "ODE installation was found in local path")

    link_libraries(
            ode
    )

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