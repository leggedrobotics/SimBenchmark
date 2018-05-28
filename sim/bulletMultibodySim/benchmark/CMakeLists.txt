include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(BtMbAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalBenchmark ${BtMbSimLink})