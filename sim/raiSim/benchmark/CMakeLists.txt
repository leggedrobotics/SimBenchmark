include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(RaiAnymalPDBenchmark
        AnymalPDBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalPDBenchmark.hpp)
target_link_libraries(RaiAnymalPDBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp)
target_link_libraries(RaiAnymalMomentumBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp)
target_link_libraries(RaiAnymalEnergyBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(RaiRollingBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiKaplaBenchmark
        KaplaBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/KaplaBenchmark.hpp)
target_link_libraries(RaiKaplaBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(Rai666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Rai666Benchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiElastic666Benchmark
        Elastic666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/Elastic666Benchmark.hpp)
target_link_libraries(RaiElastic666Benchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(RaiBouncingBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})