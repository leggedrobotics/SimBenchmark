include_directories(${CMAKE_SOURCE_DIR}/benchmark)

add_executable(BtMbAnymalBenchmark
        AnymalBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalBenchmark ${BtMbSimLink})

add_executable(BtMbAnymalMomentumBenchmark
        AnymalMomentumBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalMomentumBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalMomentumBenchmark ${BtMbSimLink})

add_executable(BtMbAnymalEnergyBenchmark
        AnymalEnergyBenchmark.cpp
        ${CMAKE_SOURCE_DIR}/benchmark/AnymalEnergyBenchmark.hpp
        ${CMAKE_SOURCE_DIR}/benchmark/BtMbBenchmark.hpp)
target_link_libraries(BtMbAnymalEnergyBenchmark ${BtMbSimLink})
