message(STATUS "  mujocoSim Demos")

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")


add_executable(mujocoSimDemo MjcSimDemo.cpp)
target_link_libraries(mujocoSimDemo mujocoSim)

add_executable(mujocoAnymalDemo AnymalDemo.cpp)
target_link_libraries(mujocoAnymalDemo mujocoSim)

add_executable(mujocoAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(mujocoAnymalEnergyDemo mujocoSim)

add_executable(mujocoAtlasDemo AtlasDemo.cpp)
target_link_libraries(mujocoAtlasDemo mujocoSim)

add_executable(mujocoBuildingDemo BuildingDemo.cpp)
target_link_libraries(mujocoBuildingDemo mujocoSim)

add_executable(mujocoHumanoidDemo HumanoidDemo.cpp)
target_link_libraries(mujocoHumanoidDemo mujocoSim)