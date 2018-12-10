message(STATUS "Build demos:")

if(BULLET AND TARGET bulletSim)
    add_subdirectory(bulletSim)
endif()

if(ODE AND TARGET odeSim)
    add_subdirectory(odeSim)
endif()

if(MUJOCO AND TARGET mujocoSim)
    add_subdirectory(mujocoSim)
endif()

if(DART AND TARGET dartSim)
    add_subdirectory(dartSim)
endif()