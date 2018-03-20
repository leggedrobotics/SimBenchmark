set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")

if(BENCHMARK_DART AND DART_FOUND)
    message(STATUS "DART installation was found in local path")

    set(DartSim_SRCS
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
            object/CheckerBoard.hpp)

    add_library(DartSimLib ${DartSim_SRCS})
endif()