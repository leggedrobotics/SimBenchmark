########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native -march=native")


########################################################################################################################
# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


########################################################################################################################
# executable

add_executable(bulletAnymalPDBenchmark AnymalPDBenchmark.cpp)
target_link_libraries(bulletAnymalPDBenchmark bulletSim benchmark)

add_executable(bulletAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(bulletAnymalMomentumBenchmark bulletSim benchmark)

add_executable(bulletAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
target_link_libraries(bulletAnymalEnergyBenchmark bulletSim benchmark)

add_executable(bulletBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(bulletBouncingBenchmark bulletSim benchmark)

add_executable(bulletRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(bulletRollingBenchmark bulletSim benchmark)

add_executable(bullet666Benchmark 666Benchmark.cpp)
target_link_libraries(bullet666Benchmark bulletSim benchmark)

add_executable(bulletElastic666Benchmark Elastic666Benchmark.cpp)
target_link_libraries(bulletElastic666Benchmark bulletSim benchmark)

add_executable(bulletKaplaBenchmark KaplaBenchmark.cpp)
target_link_libraries(bulletKaplaBenchmark bulletSim benchmark)

add_executable(bulletAtlasContactBenchmark AtlasContactBenchmark.cpp)
target_link_libraries(bulletAtlasContactBenchmark bulletSim benchmark)