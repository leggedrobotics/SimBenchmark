include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(OdeAnymalPDBenchmark
        AnymalPDBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalPDBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalPDBenchmark ${OdeSimLink})

add_executable(OdeAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalMomentumBenchmark ${OdeSimLink})

add_executable(OdeAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalEnergyBenchmark ${OdeSimLink})

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