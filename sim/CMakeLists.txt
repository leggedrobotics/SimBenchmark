set(Benchmark_SRCS
        common/UserHandle.hpp
        common/World_RG.hpp
        common/World_RG.cpp
        common/WorldInterface.hpp
        common/SingleBodyObjectInterface.hpp
        common/ArticulatedSystem.hpp
        common/math.hpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})

if(BENCHMARK_RAISIM)
    message(STATUS "Benchmark raiSim")
    add_subdirectory(raiSim)
endif(BENCHMARK_RAISIM)

if(BENCHMARK_BULLET)
    message(STATUS "Benchmark Bullet")
    add_subdirectory(bulletSim)
endif(BENCHMARK_BULLET)

if(BENCHMARK_ODE)
    add_subdirectory(odeSim)
endif(BENCHMARK_ODE)

if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark MUJOCO")
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(BENCHMARK_DART)
    add_subdirectory(dartSim)
endif(BENCHMARK_DART)