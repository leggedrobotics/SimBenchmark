add_subdirectory(common)

if(BENCHMARK_RAISIM)
    message(STATUS "Benchmark raiSim")
    add_subdirectory(raiSim)
endif(BENCHMARK_RAISIM)

if(BULLET)
    message(STATUS "Benchmark Bullet")
    add_subdirectory(bulletSim)
endif(BULLET)

if(ODE)
    message(STATUS "Benchmark ODE")
    add_subdirectory(odeSim)
endif(ODE)

if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark MUJOCO")
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(DART)
    message(STATUS "Benchmark DART")
    add_subdirectory(dartSim)
endif(DART)