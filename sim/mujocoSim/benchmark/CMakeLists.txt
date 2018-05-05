include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(MjcAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/MjcBenchmark.hpp
        )
target_link_libraries(MjcAnymalBenchmark ${MjcSimLink})

add_executable(MjcRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(MjcRollingBenchmark ${MjcSimLink})