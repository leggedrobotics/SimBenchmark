include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(DartAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/DartBenchmark.hpp
        )
target_link_libraries(DartAnymalBenchmark ${DartSimLink})

add_executable(DartRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(DartRollingBenchmark ${DartSimLink})

add_executable(DartBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(DartBuildingBenchmark ${DartSimLink})

add_executable(Dart666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Dart666Benchmark ${DartSimLink})

add_executable(DartBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(DartBouncingBenchmark ${DartSimLink})