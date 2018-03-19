set(Benchmark_SRCS
        base/UserHandle.hpp
        base/World_RG.hpp
        base/World_RG.cpp
        base/SingleBodyObject.hpp
        base/SingleBodyObject.cpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})

# sims
add_subdirectory(bulletSim)
add_subdirectory(odeSim)
add_subdirectory(mujocoSim)
add_subdirectory(dartSim)