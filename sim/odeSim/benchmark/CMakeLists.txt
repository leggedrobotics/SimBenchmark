include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(OdeAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/OdeBenchmark.hpp)
target_link_libraries(OdeAnymalBenchmark ${OdeSimLink})

add_executable(OdeRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(OdeRollingBenchmark ${OdeSimLink})

add_executable(OdeBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(OdeBuildingBenchmark ${OdeSimLink})

add_executable(OdeThousandBenchmark
        ThousandBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/ThousandBenchmark.hpp)
target_link_libraries(OdeThousandBenchmark ${OdeSimLink})