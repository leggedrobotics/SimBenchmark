message(STATUS "Build mujocoSim")

########################################################################################################################
# dependencies
externalproject_get_property(ex_mujoco SOURCE_DIR)
set(mujoco_include_dir "${SOURCE_DIR}/include/")
set(mujoco_bin_dir "${SOURCE_DIR}/bin/")

find_library(glew NAMES glew
        PATHS ${mujoco_bin_dir})
find_library(glewegl NAMES glewegl
        PATHS ${mujoco_bin_dir})
find_library(glewosmesa NAMES glewosmesa
        PATHS ${mujoco_bin_dir})
find_library(glfw NAMES glfw libglfw.so.3
        PATHS ${mujoco_bin_dir})
find_library(mujoco200 NAMES mujoco200
        PATHS ${mujoco_bin_dir})
find_library(mujoco200nogl NAMES mujoco200nogl
        PATHS ${mujoco_bin_dir})


########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

set(MUJOCO_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/mjpro150)

include_directories(${MUJOCO_LIB_DIR}/include)


########################################################################################################################
# files
find_file(mujocoKey
        mjkey.txt
        PATH "${CMAKE_SOURCE_DIR}/res/mujoco"
        )

if(mujocoKey STREQUAL mujocoKey-NOTFOUND)
    message(FATAL_ERROR
            "Cannot find mjkey.txt from res/mujoco/ \n"
            "Put your mjkey.txt into res/mujoco/ directory")
else()
    message(STATUS "Found mjkey.txt file. Copying the file into build directory")
endif()


file(INSTALL ${mujocoKey}
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


########################################################################################################################
# source
include_directories(src)

set(mjcSimSrc
        src/MjcWorld.cpp
        src/MjcWorld.hpp
        src/MjcSim.cpp
        src/MjcSim.hpp
        src/UserHandle.hpp
        src/object/MjcSingleBodyObject.cpp
        src/object/MjcSingleBodyObject.hpp
        src/object/MjcSphere.cpp
        src/object/MjcSphere.hpp
        src/object/MjcBox.cpp
        src/object/MjcBox.hpp
        src/object/MjcCapsule.cpp
        src/object/MjcCapsule.hpp
        src/object/MjcCheckerBoard.cpp
        src/object/MjcCheckerBoard.hpp
        src/object/MjcCylinder.cpp
        src/object/MjcCylinder.hpp)


########################################################################################################################
# library
add_library(mujocoSim
        STATIC
        ${mjcSimSrc})

add_dependencies(mujocoSim
        ex_mujoco
        ex_raigraphics
        commonSim
        )

target_include_directories(mujocoSim
        PUBLIC
        src
        ${mujoco_include_dir}
        ${raigraphics_INCLUDE_DIRS}
        )

target_link_libraries(mujocoSim
        ${mujoco200nogl}
        commonSim
        #        ${glew}
        #        ${glfw}
        #        ${mujoco150}
        #        GL
        #        GLU
        #        glut
        )


########################################################################################################################
# subdirectory
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)


if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)

#if(BUILD_UTIL)
#    add_subdirectory(test)
#endif(BUILD_TEST)
