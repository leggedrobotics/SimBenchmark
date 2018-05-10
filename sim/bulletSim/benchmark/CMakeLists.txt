include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(BtAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtBenchmark.hpp)
target_link_libraries(BtAnymalBenchmark ${BtSimLink})

add_executable(BtRollingBenchmark
        RollingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/RollingBenchmark.hpp)
target_link_libraries(BtRollingBenchmark ${BtSimLink})

add_executable(BtBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(BtBuildingBenchmark ${BtSimLink})

add_executable(BtThousandBenchmark
        ThousandBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/ThousandBenchmark.hpp)
target_link_libraries(BtThousandBenchmark ${BtSimLink})