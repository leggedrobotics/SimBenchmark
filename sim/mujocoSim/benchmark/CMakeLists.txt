include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(MjcAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalBenchmark ${MjcSimLink})

add_executable(MjcAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalMomentumBenchmark ${MjcSimLink})

add_executable(MjcRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(MjcRollingBenchmark ${MjcSimLink})

add_executable(MjcBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(MjcBuildingBenchmark ${MjcSimLink})

add_executable(Mjc666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Mjc666Benchmark ${MjcSimLink})

add_executable(MjcBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(MjcBouncingBenchmark ${MjcSimLink})