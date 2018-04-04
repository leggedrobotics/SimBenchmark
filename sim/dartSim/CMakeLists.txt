set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")

# dart (optional)
find_package(DART CONFIG REQUIRED)
include_directories(${DART_INCLUDE_DIRS})

# source
include_directories(src)

set(DartSim_SRCS
        src/World.cpp
        src/World.hpp
        src/World_RG.cpp
        src/World_RG.hpp
        src/UserHandle.hpp
        src/object/SingleBodyObject.cpp
        src/object/SingleBodyObject.hpp
        src/object/Sphere.cpp
        src/object/Sphere.hpp
        src/object/Box.cpp
        src/object/Box.hpp
        src/object/Capsule.cpp
        src/object/Capsule.hpp
        src/object/CheckerBoard.cpp
        src/object/CheckerBoard.hpp
        src/object/Object.cpp
        src/object/Object.hpp
        src/object/Cylinder.cpp
        src/object/Cylinder.hpp)

# library
add_library(DartSimLib ${DartSim_SRCS})
target_link_libraries(DartSimLib
        ${DART_LIBRARIES}
        )

# link
set(DartSimLink
        DartSimLib
        ${BENCHMARK_LINK}
        )

# sub-directories
add_subdirectory(demo)