include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(RaiAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp)
target_link_libraries(RaiAnymalBenchmark ${RAI_SIM_LINK})