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

set(MUJOCO_SIM_SRC
        src/World.cpp
        src/World.hpp
        src/World_RG.cpp
        src/World_RG.hpp
        src/UserHandle.hpp
        src/object/SingleBodyObjectInterface.cpp
        src/object/SingleBodyObject.hpp
        src/object/Sphere.cpp
        src/object/Sphere.hpp
        src/object/Box.cpp
        src/object/Box.hpp
        src/object/Capsule.cpp
        src/object/Capsule.hpp
        src/object/CheckerBoard.cpp
        src/object/CheckerBoard.hpp
        )

# library
add_library(MUJOCO_SIM_LIB ${MUJOCO_SIM_SRC})
target_link_libraries(MUJOCO_SIM_LIB
        ${glew}
        ${glfw}
        ${mujoco150}
        ${mujoco150nogl}
        GL
        GLU
        glut)

set(MUJOCO_SIM_LINK
        MUJOCO_SIM_LIB
        ${BENCHMARK_LINK}
        )

# subdirectory
add_subdirectory(test)
add_subdirectory(demo)
