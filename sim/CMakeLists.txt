add_subdirectory(common)

if(BENCHMARK_RAISIM)
    add_subdirectory(raiSim)
endif(BENCHMARK_RAISIM)

if(BULLET)
    add_subdirectory(bulletSim)
endif(BULLET)

if(ODE)
    add_subdirectory(odeSim)
endif(ODE)

if(BENCHMARK_MUJOCO)
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(DART)
    add_subdirectory(dartSim)
endif(DART)