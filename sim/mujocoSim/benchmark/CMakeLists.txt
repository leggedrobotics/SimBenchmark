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

add_executable(MjcBuildingBenchmark
        BuildingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BuildingBenchmark.hpp)
target_link_libraries(MjcBuildingBenchmark ${MjcSimLink})

add_executable(MjcThousandBenchmark
        ThousandBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/ThousandBenchmark.hpp)
target_link_libraries(MjcThousandBenchmark ${MjcSimLink})

add_executable(MjcBouncingBenchmark
        BouncingBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/BouncingBenchmark.hpp)
target_link_libraries(MjcBouncingBenchmark ${MjcSimLink})