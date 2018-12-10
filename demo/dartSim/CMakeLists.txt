message(STATUS "  dartSim Demos")


set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")


add_executable(dartZeroGravityDemo ZeroGravityDemo.cpp)
target_link_libraries(dartZeroGravityDemo dartSim)

add_executable(dartAnymalDemo AnymalDemo.cpp)
target_link_libraries(dartAnymalDemo dartSim)

add_executable(dartAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(dartAnymalEnergyDemo dartSim)

add_executable(dartAtlasDemo AtlasDemo.cpp)
target_link_libraries(dartAtlasDemo dartSim)

add_executable(dartRigidBodyDemo RigidBodyDemo.cpp)
target_link_libraries(dartRigidBodyDemo dartSim)

add_executable(dartBuildingDemo BuildingDemo.cpp)
target_link_libraries(dartBuildingDemo dartSim)

add_executable(dartStackDemo StackDemo.cpp)
target_link_libraries(dartStackDemo dartSim)