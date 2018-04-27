include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(OdeAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp
        )
target_link_libraries(OdeAnymalBenchmark ${OdeSimLink})
