set(DartSim_SRCS
        World.cpp
        World.hpp World_RG.cpp World_RG.hpp object/Sphere.cpp object/Sphere.hpp object/SingleBodyObject.cpp object/SingleBodyObject.hpp object/Box.cpp object/Box.hpp object/Capsule.cpp object/Capsule.hpp object/CheckerBoard.cpp object/CheckerBoard.hpp)

add_library(DartSim ${DartSim_SRCS})
