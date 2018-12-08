########################################################################################################################
# dependencies
find_package(raigraphics REQUIRED)

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

add_library(commonSim ${CommonSrc})

#add_dependencies(commonSim
#        ex_raigraphics
#        )

target_include_directories(commonSim
        PUBLIC
        ${raigraphics_INCLUDE_DIRS})

target_link_libraries(commonSim
        ${raigraphics_LINK}
        )
