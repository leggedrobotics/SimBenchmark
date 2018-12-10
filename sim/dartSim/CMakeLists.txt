########################################################################################################################
# dependencies
externalproject_get_property(ex_dart INSTALL_DIR)
find_package(DART
        CONFIG
        NO_DEFAULT_PATH
        PATHS           "${INSTALL_DIR}"
        COMPONENTS
        utils-urdf
        collision-bullet
        collision-ode
        )

externalproject_get_property(ex_raigraphics INSTALL_DIR)
find_package(raigraphics
        CONFIG
        NO_DEFAULT_PATH
        PATHS           "${INSTALL_DIR}"
        )


########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")


########################################################################################################################
# source
set(DartSim_SRCS
        src/DartWorld.cpp
        src/DartWorld.hpp
        src/DartSim.cpp
        src/DartSim.hpp
        src/UserHandle.hpp
        src/object/DartSingleBodyObject.cpp
        src/object/DartSingleBodyObject.hpp
        src/object/DartSphere.cpp
        src/object/DartSphere.hpp
        src/object/DartBox.cpp
        src/object/DartBox.hpp
        src/object/DartCapsule.cpp
        src/object/DartCapsule.hpp
        src/object/DartCheckerBoard.cpp
        src/object/DartCheckerBoard.hpp
        src/object/DartObject.cpp
        src/object/DartObject.hpp
        src/object/DartCylinder.cpp
        src/object/DartCylinder.hpp
        src/object/DartArticulatedSystem.cpp
        src/object/DartArticulatedSystem.hpp)


########################################################################################################################
# library
message(STATUS "  Build dartSim")

add_library(dartSim
        STATIC
        ${DartSim_SRCS}
        )

add_dependencies(dartSim
        ex_dart
        ex_raigraphics
        commonSim
        )

target_include_directories(dartSim
        PUBLIC
        src
        ${DART_INCLUDE_DIRS}
        )

target_link_libraries(dartSim
        ${DART_LIBRARIES}
        )


########################################################################################################################
# sub-directories
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)