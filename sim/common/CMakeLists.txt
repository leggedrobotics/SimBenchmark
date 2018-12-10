message(STATUS "  commonSim for common parts")

########################################################################################################################
# dependencies
externalproject_get_property(ex_raigraphics INSTALL_DIR)
find_package(raigraphics
        CONFIG
        NO_DEFAULT_PATH
        PATHS           "${INSTALL_DIR}"
        )


########################################################################################################################
# source
set(CommonSrc
        UserHandle.hpp
        WorldRG.hpp
        WorldRG.cpp
        Configure.hpp
        math.hpp
        interface/CheckerboardInterface.hpp
        interface/ArticulatedSystemInterface.hpp
        interface/WorldInterface.hpp
        interface/SingleBodyObjectInterface.hpp
        )

########################################################################################################################
# library
add_library(commonSim ${CommonSrc})

add_dependencies(commonSim
        ex_raigraphics
        )

target_include_directories(commonSim
        PUBLIC
        ${raigraphics_INCLUDE_DIRS}
        )

target_link_libraries(commonSim
        ${raigraphics_LINK}
        )
