########################################################################################################################
# flags
#set(CMAKE_CXX_FLAGS_DEBUG
#        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -DODESIM_DEBUG")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")


########################################################################################################################
# ode double precision
add_definitions(-DODE_DOUBLE_PRECISION)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

########################################################################################################################
# executable

add_executable(odeAnymalPDBenchmark AnymalPDBenchmark.cpp)
target_link_libraries(odeAnymalPDBenchmark odeSim benchmark)

add_executable(odeAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(odeAnymalMomentumBenchmark odeSim benchmark)

add_executable(odeAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
target_link_libraries(odeAnymalEnergyBenchmark odeSim benchmark)

add_executable(odeRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(odeRollingBenchmark odeSim benchmark)

add_executable(odeKaplaBenchmark KaplaBenchmark.cpp)
target_link_libraries(odeKaplaBenchmark odeSim benchmark)

add_executable(ode666Benchmark 666Benchmark.cpp)
target_link_libraries(ode666Benchmark odeSim benchmark)

add_executable(odeElastic666Benchmark Elastic666Benchmark.cpp)
target_link_libraries(odeElastic666Benchmark odeSim benchmark)

add_executable(odeBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(odeBouncingBenchmark odeSim benchmark)