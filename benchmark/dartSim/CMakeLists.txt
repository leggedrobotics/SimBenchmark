########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")


########################################################################################################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


########################################################################################################################
# executable

add_executable(dartAnymalPDBenchmark AnymalPDBenchmark.cpp)
target_link_libraries(dartAnymalPDBenchmark dartSim benchmark)

add_executable(dartAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(dartAnymalMomentumBenchmark dartSim benchmark)

add_executable(dartAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
target_link_libraries(dartAnymalEnergyBenchmark dartSim benchmark)

add_executable(dartRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(dartRollingBenchmark dartSim benchmark)

add_executable(dartKaplaBenchmark KaplaBenchmark.cpp)
target_link_libraries(dartKaplaBenchmark dartSim benchmark)

add_executable(dart666Benchmark 666Benchmark.cpp)
target_link_libraries(dart666Benchmark dartSim benchmark)

add_executable(dartElastic666Benchmark Elastic666Benchmark.cpp)
target_link_libraries(dartElastic666Benchmark dartSim benchmark)

add_executable(dartBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(dartBouncingBenchmark dartSim benchmark)

add_executable(dartAtlasContactBenchmark AtlasContactBenchmark.cpp)
target_link_libraries(dartAtlasContactBenchmark dartSim benchmark)