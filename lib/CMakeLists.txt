# raiCommons
find_library(raiCommonGraph
        NAMES rai_common_graph
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/raiCommon)
find_library(raiCommonLogger
        NAMES rai_common_logger
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/raiCommon)
find_library(raiCommonTimer
        NAMES rai_common_timer
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/raiCommon)
set(RAI_COMMON_LINK
        ${raiCommonGraph}
        ${raiCommonLogger}
        ${raiCommonTimer}
        PARENT_SCOPE)

# raiGraphics
set(RAI_GRAPHICS_OPENGL_LINK
        ${CMAKE_CURRENT_SOURCE_DIR}/raiGraphics/libraiGraphicsOpenglLib.so
        GL GLEW SDL2 SDL2_ttf assimp SOIL pthread
        ${RAI_COMMON_LINK}
        PARENT_SCOPE)

# yaml-cpp
#find_package(yaml-cpp REQUIRED)

set(YAML_CPP_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/libyaml-cpp.a
        PARENT_SCOPE)
message("${YAML_CPP_LIBRARIES}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include)

# bullet: sim/bulletMultibodySim
# ode: sim/odeSim