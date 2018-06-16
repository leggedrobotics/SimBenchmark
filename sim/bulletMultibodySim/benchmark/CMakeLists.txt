include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(BtMbAnymalPDBenchmark
        AnymalPDBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalPDBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalPDBenchmark ${BtMbSimLink})

add_executable(BtMbAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalMomentumBenchmark ${BtMbSimLink})

add_executable(BtMbAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalEnergyBenchmark ${BtMbSimLink})

add_executable(BtMbBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbBouncingBenchmark ${BtMbSimLink})

add_executable(BtMbRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbRollingBenchmark ${BtMbSimLink})

add_executable(BtMb666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMb666Benchmark ${BtMbSimLink})

add_executable(BtMbBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbBuildingBenchmark ${BtMbSimLink})