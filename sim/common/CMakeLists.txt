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

add_library(CommonLib ${CommonSrc})