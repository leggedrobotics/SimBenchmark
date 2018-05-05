include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(DartAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/DartBenchmark.hpp
        )
target_link_libraries(DartAnymalBenchmark ${DartSimLink})

add_executable(DartRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(DartRollingBenchmark ${DartSimLink})