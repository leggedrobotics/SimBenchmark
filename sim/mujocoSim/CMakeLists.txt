set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

set(MUJOCO_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/mjpro150)

include_directories(${MUJOCO_LIB_DIR}/include)

# mujoco shared libraries
find_library(glew NAMES glew
        PATHS ${MUJOCO_LIB_DIR}/bin)
find_library(glewegl NAMES glewegl
        PATHS ${MUJOCO_LIB_DIR}/bin)
find_library(glewosmesa NAMES glewosmesa
        PATHS ${MUJOCO_LIB_DIR}/bin)
find_library(glfw NAMES glfw libglfw.so.3
        PATHS ${MUJOCO_LIB_DIR}/bin)
find_library(mujoco150 NAMES mujoco150
        PATHS ${MUJOCO_LIB_DIR}/bin)
find_library(mujoco150nogl NAMES mujoco150nogl
        PATHS ${MUJOCO_LIB_DIR}/bin)

# files
file(INSTALL ${MUJOCO_LIB_DIR}/mjkey.txt
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(INSTALL ${MUJOCO_LIB_DIR}/model
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# source
include_directories(src)

set(MjcSimSrc
        src/MjcWorld.cpp
        src/MjcWorld.hpp
        src/MjcWorld_RG.cpp
        src/MjcWorld_RG.hpp
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

# library
add_library(MjcSimLib ${MjcSimSrc})
target_link_libraries(MjcSimLib
#        ${glew}
#        ${glfw}
        ${mujoco150}
#        ${mujoco150nogl}
#        GL
#        GLU
#        glut)
        )

set(MjcSimLink
        MjcSimLib
        ${BENCHMARK_LINK}
        )

# subdirectory
add_subdirectory(benchmark)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)
