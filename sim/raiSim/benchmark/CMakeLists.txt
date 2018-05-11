include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(RaiAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp)
target_link_libraries(RaiAnymalBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(RaiRollingBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(RaiBuildingBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiThousandBenchmark
        ThousandBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/ThousandBenchmark.hpp)
target_link_libraries(RaiThousandBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})

add_executable(RaiBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(RaiBouncingBenchmark ${RAI_SIM_LINK} ${BENCHMARK_LINK})