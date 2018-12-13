########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -Wl,--no-as-needed -O0 -mtune=native -march=native -g")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native -fPIC")


########################################################################################################################
# collision backend
add_definitions(-DCOLLISION_BACKEND_ODE)


########################################################################################################################
# executable

add_executable(raiAnymalPDBenchmark AnymalPDBenchmark.cpp)
target_link_libraries(raiAnymalPDBenchmark raiSim raiSimODE benchmark)

add_executable(raiAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(raiAnymalMomentumBenchmark raiSim raiSimODE benchmark)

add_executable(raiAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
target_link_libraries(raiAnymalEnergyBenchmark raiSim raiSimODE benchmark)

add_executable(raiRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(raiRollingBenchmark raiSim raiSimODE benchmark)

add_executable(raiKaplaBenchmark KaplaBenchmark.cpp)
target_link_libraries(raiKaplaBenchmark raiSim raiSimODE benchmark)

add_executable(rai666Benchmark 666Benchmark.cpp)
target_link_libraries(rai666Benchmark raiSim raiSimODE benchmark)

add_executable(raiElastic666Benchmark Elastic666Benchmark.cpp)
target_link_libraries(raiElastic666Benchmark raiSim raiSimODE benchmark)

add_executable(raiBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(raiBouncingBenchmark raiSim raiSimODE benchmark)

add_executable(raiAtlasContactBenchmark AtlasContactBenchmark.cpp)
target_link_libraries(raiAtlasContactBenchmark raiSim raiSimODE benchmark)