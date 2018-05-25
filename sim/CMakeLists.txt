set(Benchmark_SRCS
        common/UserHandle.hpp
        common/WorldRG.hpp
        common/WorldRG.cpp
        common/math.hpp
#        common/interface/ArticulatedSystemInterface.hpp
#        common/interface/WorldInterface.hpp
#        common/interface/SingleBodyObjectInterface.hpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})

message(STATUS "Simulation Engines:")

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
    add_subdirectory(bulletMultibodySim)
endif(BENCHMARK_BULLET_MULTIBODY)

if(BENCHMARK_ODE)
    message(STATUS "Benchmark ODE")
    add_subdirectory(odeSim)
endif(BENCHMARK_ODE)

if(BENCHMARK_MUJOCO)
    message(STATUS "Benchmark MUJOCO")
    add_subdirectory(mujocoSim)
endif(BENCHMARK_MUJOCO)

if(BENCHMARK_DART)
    message(STATUS "Benchmark DART")
    add_subdirectory(dartSim)
endif(BENCHMARK_DART)