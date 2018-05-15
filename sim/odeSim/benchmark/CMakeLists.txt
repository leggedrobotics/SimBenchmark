include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(OdeAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalBenchmark ${OdeSimLink})

add_executable(OdeAnymalZerogBenchmark
        AnymalZerogBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalZerogBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalZerogBenchmark ${OdeSimLink})

add_executable(OdeRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(OdeRollingBenchmark ${OdeSimLink})

add_executable(OdeBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(OdeBuildingBenchmark ${OdeSimLink})

add_executable(Ode666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Ode666Benchmark ${OdeSimLink})

add_executable(OdeBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(OdeBouncingBenchmark ${OdeSimLink})