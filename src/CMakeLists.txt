set(Benchmark_SRCS
        base/UserHandle.hpp
        base/World_RG.hpp
        base/World_RG.cpp
        base/SingleBodyObject.hpp
        base/ArticulatedSystem.hpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})

# sims
if(BENCHMARK_BULLET)
    add_subdirectory(bulletSim)
endif(BENCHMARK_BULLET)

if(BENCHMARK_ODE)
    add_subdirectory(odeSim)
endif(BENCHMARK_ODE)

if(BENCHMARK_MUJOCO)
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(BENCHMARK_DART)
    add_subdirectory(dartSim)
endif(BENCHMARK_DART)