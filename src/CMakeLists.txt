add_subdirectory(bulletSim)
#add_subdirectory(odeSim)
#add_subdirectory(mujocoSim)
add_subdirectory(dartSim)

# Benchmark
set(Benchmark_SRCS
        base/UserHandle.hpp
        base/World_RG.hpp
        base/World_RG.cpp
        base/SingleBodyObject.hpp
        base/SingleBodyObject.cpp
        )

add_library(BenchmarkLib ${Benchmark_SRCS})
set(BENCHMARK_LINK
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        BenchmarkLib
        )