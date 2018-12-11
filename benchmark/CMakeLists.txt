########################################################################################################################
# dependencies

# yaml-cpp
externalproject_get_property(ex_yamlcpp INSTALL_DIR)
set(yamlcpp_include_dir "${INSTALL_DIR}/include")
set(yamlcpp_lib_dir "${INSTALL_DIR}/lib")

find_library(yaml-cpp
        NO_DEFAULT_PATH
        NAMES       yaml-cpp
        PATHS       ${yamlcpp_lib_dir})

#include_directories(${yamlcpp_include_dir})


########################################################################################################################
# source
set(benchmarkHeaders
        "include/666Benchmark.hpp"
        "include/AnymalEnergyBenchmark.hpp"
        "include/AnymalMomentumBenchmark.hpp"
        "include/AnymalPDBenchmark.hpp"
        "include/AtlasContactBenchmark.hpp"
        "include/BenchmarkTest.hpp"
        "include/BouncingBenchmark.hpp"
        "include/Elastic666Benchmark.hpp"
        "include/KaplaBenchmark.hpp"
        "include/RollingBenchmark.hpp"
        )

########################################################################################################################
# macro variable

add_definitions(-DBENCHMARKROOTPATH="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DBENCHMARKYAMLPATH="${CMAKE_CURRENT_SOURCE_DIR}/res/yaml")

########################################################################################################################
# library

# this library is header only (interface)
add_library(benchmark INTERFACE)

add_dependencies(benchmark
        ex_yamlcpp
        Boost
        )

target_include_directories(benchmark
        INTERFACE
        "include"
        ${yamlcpp_include_dir}
        )

target_link_libraries(benchmark
        INTERFACE
        ${yaml-cpp}
        ${Boost_LIBRARIES}
        )

########################################################################################################################
# scripts

file(COPY res/shell/anymal-pd.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/rolling.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/kapla.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/666.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/bouncing.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/666-elastic.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/anymal-momentum.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/anymal-energy.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/atlas-contact.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/selectsim.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(sim.sh.in ${CMAKE_CURRENT_BINARY_DIR}/sim.sh @ONLY)


########################################################################################################################
# sub-directories

message(STATUS "Build benchmark tests:")

if(TARGET bulletSim)
    add_subdirectory(bulletSim)
else()
    message(STATUS "  Cannot find bulletSim. Stop build bulletSim benchmark tests")
endif()

if(TARGET odeSim)
    add_subdirectory(odeSim)
else()
    message(STATUS "  Cannot find odeSim. Stop build odeSim benchmark tests")
endif()

if(TARGET mujocoSim)
    add_subdirectory(mujocoSim)
else()
    message(STATUS "  Cannot find mujocoSim. Stop build mujocoSim benchmark tests")
endif()

if(TARGET dartSim)
    add_subdirectory(dartSim)
else()
    message(STATUS "  Cannot find dartSim. Stop build dartSim benchmark tests")
endif()

### --------------------------- export files ---------------------------
#configure_file(shell/rolling-benchmark.sh.in ${CMAKE_CURRENT_BINARY_DIR}/rolling-benchmark.sh @ONLY)
#configure_file(shell/bounce.sh.in ${CMAKE_CURRENT_BINARY_DIR}/bounce.sh @ONLY)
#
## mujoco model scripts
#file(COPY mujoco
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#
## parameter yaml
#file(COPY rolling-benchmark.yaml
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})