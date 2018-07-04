include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(MjcAnymalPDBenchmark
        AnymalPDBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalPDBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalPDBenchmark ${MjcSimLink})

add_executable(MjcAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalMomentumBenchmark ${MjcSimLink})

add_executable(MjcAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalEnergyBenchmark ${MjcSimLink})

add_executable(MjcRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(MjcRollingBenchmark ${MjcSimLink})

add_executable(MjcKaplaBenchmark
        KaplaBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/KaplaBenchmark.hpp)
target_link_libraries(MjcKaplaBenchmark ${MjcSimLink})

add_executable(Mjc666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Mjc666Benchmark ${MjcSimLink})

add_executable(MjcBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(MjcBouncingBenchmark ${MjcSimLink})

add_executable(MjcAtlasContactBenchmark
        AtlasContactBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AtlasContactBenchmark.hpp)
target_link_libraries(MjcAtlasContactBenchmark ${MjcSimLink})