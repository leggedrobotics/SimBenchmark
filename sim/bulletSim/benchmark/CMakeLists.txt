include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(BtAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp)
target_link_libraries(BtAnymalBenchmark ${BULLET_SIM_LINK})
