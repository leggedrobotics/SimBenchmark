message(STATUS "  raiSim for raiSim")

########################################################################################################################
# dependencies

# raicommon
externalproject_get_property(ex_raicommon_for_raisim INSTALL_DIR)
set(raicommon_include_dir "${INSTALL_DIR}/include")
set(raicommon_lib_dir "${INSTALL_DIR}/lib")

find_library(raicommon_graph
        NAMES           rai_common_graph
        PATHS           "${raicommon_lib_dir}"
        NO_DEFAULT_PATH
        )

find_library(raicommon_logger
        NAMES           rai_common_logger
        PATHS           "${raicommon_lib_dir}"
        NO_DEFAULT_PATH
        )

find_library(raicommon_timer
        NAMES           rai_common_timer
        PATHS           "${raicommon_lib_dir}"
        NO_DEFAULT_PATH
        )

# raigraphics
externalproject_get_property(ex_raigraphics_for_raisim INSTALL_DIR)
externalproject_get_property(ex_raigraphics_for_raisim SOURCE_DIR)

set(raigraphics_include_dir "${SOURCE_DIR}/include")
set(raigraphics_lib_dir "${INSTALL_DIR}/lib")

find_library(raiGraphicsOpengl
        NAMES           raiGraphicsOpenglLib
        PATHS           "${raigraphics_lib_dir}"
        NO_DEFAULT_PATH
        )

# raiSim
externalproject_get_property(ex_raisim INSTALL_DIR)
set(raisim_install_dir ${INSTALL_DIR})

find_package(raiSim
        CONFIG
        QUIET
        PATHS           "${raisim_install_dir}"
        NO_DEFAULT_PATH
        )

if(NOT raiGraphicsOpengl OR NOT raiSim_FOUND)
    message(WARNING "Cannot find dependency libs. Stop build raiSim")
    return()
endif()


########################################################################################################################
# set raiSim backend

if(RAISIM_BACKEND STREQUAL "ODE")
    add_definitions(-DCOLLISION_BACKEND=ODE)
elseif(RAISIM_BACKEND STREQUAL "BULLET")
    add_definitions(-DCOLLISION_BACKEND=BULLET)
else()
    message(FATAL_ERROR "Unsupported RAISIM_BACKEND option. Should be ODE or BULLET")
endif()


########################################################################################################################
# library

# raiSim
add_library(raiSim
        SHARED
        IMPORTED
        GLOBAL
        )

add_dependencies(raiSim
        ex_raiSim
        ex_raigraphics_for_raisim
        )

set_target_properties(raiSim
        PROPERTIES
        IMPORTED_LOCATION
        "${raisim_install_dir}/lib/libraiSimLib.so"
        INTERFACE_INCLUDE_DIRECTORIES
        "${RAI_SIM_INCLUDE_DIR}"
        )

target_compile_definitions(raiSim
        INTERFACE
        "-DCOLLISION_BACKEND=ODE"
        )

target_include_directories(raiSim
        INTERFACE
        ${raicommon_include_dir}
        ${raigraphics_include_dir}
        )

target_link_libraries(raiSim
        INTERFACE
        ${raicommon_graph}
        ${raicommon_logger}
        ${raicommon_timer}
        ${raiGraphicsOpengl}
        ${Boost_LIBRARIES}
        GL GLEW SDL2 SDL2_ttf assimp SOIL pthread
        )

# raiSimODE
add_library(raiSimODE
        STATIC
        IMPORTED
        GLOBAL
        )

add_dependencies(raiSimODE
        ex_raiSim
        ex_raigraphics_for_raisim
        )

set_target_properties(raiSimODE
        PROPERTIES
        IMPORTED_LOCATION
        "${raisim_install_dir}/lib/libraiSimODE.a"
        INTERFACE_INCLUDE_DIRECTORIES
        "${RAI_SIM_INCLUDE_DIR}"
        )


########################################################################################################################
# sub-directories
if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)
