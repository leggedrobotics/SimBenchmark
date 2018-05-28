include_directories(${CMAKE_SOURCE_DIR}/benchmark)

#add_executable(BtAnymalBenchmark
#        AnymalBenchmark.cpp
#        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
#        ${CMAKE_SOURCE_DIR}/benchmark/BtBenchmark.hpp)
#target_link_libraries(BtAnymalBenchmark ${BtSimLink})

add_executable(BtAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtBenchmark.hpp)
target_link_libraries(BtAnymalMomentumBenchmark ${BtSimLink})

add_executable(BtAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtBenchmark.hpp)
target_link_libraries(BtAnymalEnergyBenchmark ${BtSimLink})

add_executable(BtRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(BtRollingBenchmark ${BtSimLink})

add_executable(BtBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(BtBuildingBenchmark ${BtSimLink})

add_executable(Bt666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Bt666Benchmark ${BtSimLink})

add_executable(BtBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(BtBouncingBenchmark ${BtSimLink})