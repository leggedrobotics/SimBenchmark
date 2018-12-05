add_subdirectory(common)

if(BENCHMARK_RAISIM)
    message(STATUS "Benchmark raiSim")
    add_subdirectory(raiSim)
endif(BENCHMARK_RAISIM)

if(BENCHMARK_BULLET)
    message(STATUS "Benchmark Bullet")
    add_subdirectory(bulletSim)
endif(BENCHMARK_BULLET)

if(BENCHMARK_BULLET_MULTIBODY)
    message(STATUS "Benchmark Bullet Multibody")
    add_subdirectory(bulletSim)
endif(BENCHMARK_BULLET_MULTIBODY)

if(ODE)
    message(STATUS "Benchmark ODE")
    add_subdirectory(odeSim)
endif(ODE)

if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark MUJOCO")
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(BENCHMARK_DART)
    message(STATUS "Benchmark DART")
    add_subdirectory(dartSim)
endif(BENCHMARK_DART)