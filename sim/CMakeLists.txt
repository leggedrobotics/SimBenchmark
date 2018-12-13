message(STATUS "Build Sim libraries for the following engines:")

add_subdirectory(common)

if(RAISIM)
    add_subdirectory(raiSim)
endif(RAISIM)

if(BULLET)
    add_subdirectory(bulletSim)
endif(BULLET)

if(ODE)
    add_subdirectory(odeSim)
endif(ODE)

if(MUJOCO)
    add_subdirectory(mujocoSim)
endif(MUJOCO)

if(DART)
    add_subdirectory(dartSim)
endif(DART)