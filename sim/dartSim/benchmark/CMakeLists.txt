include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(DartAnymalPDBenchmark
        AnymalPDBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalPDBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/DartBenchmark.hpp
        )
target_link_libraries(DartAnymalPDBenchmark ${DartSimLink})

add_executable(DartAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/DartBenchmark.hpp
        )
target_link_libraries(DartAnymalMomentumBenchmark ${DartSimLink})

add_executable(DartAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/DartBenchmark.hpp
        )
target_link_libraries(DartAnymalEnergyBenchmark ${DartSimLink})

add_executable(DartRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(DartRollingBenchmark ${DartSimLink})

add_executable(DartKaplaBenchmark
        KaplaBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/KaplaBenchmark.hpp)
target_link_libraries(DartKaplaBenchmark ${DartSimLink})

add_executable(Dart666Benchmark
        666Benchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
target_link_libraries(Dart666Benchmark ${DartSimLink})

#add_executable(DartElastic666Benchmark
#        Elastic666Benchmark.cpp
#        ${CMAKE_SOURCE_DIR}/benchmark/666Benchmark.hpp)
#target_link_libraries(DartElastic666Benchmark ${DartSimLink})

add_executable(DartBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(DartBouncingBenchmark ${DartSimLink})