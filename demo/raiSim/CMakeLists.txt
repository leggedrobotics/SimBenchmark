message(STATUS "  raiSim Demos")


set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -Wl,--no-as-needed -O0 -mtune=native -march=native -g")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native -fPIC")


########################################################################################################################
# collision backend
add_definitions(-DCOLLISION_BACKEND_ODE)


########################################################################################################################
# executable

add_executable(raiStackDemo StackDemo.cpp)
target_link_libraries(raiStackDemo raiSim raiSimODE)

add_executable(raiAnymalDemo AnymalDemo.cpp)
target_link_libraries(raiAnymalDemo raiSim raiSimODE)

add_executable(raiAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(raiAnymalEnergyDemo raiSim raiSimODE)

add_executable(raiAtlasDemo AtlasDemo.cpp)
target_link_libraries(raiAtlasDemo raiSim raiSimODE)

add_executable(raiMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(raiMultiBodyDemo raiSim raiSimODE)

add_executable(raiRobotArmDemo RobotArmDemo.cpp)
target_link_libraries(raiRobotArmDemo raiSim raiSimODE)

add_executable(raiBuildingDemo BuildingDemo.cpp)
target_link_libraries(raiBuildingDemo raiSim raiSimODE)

add_executable(raiElasticCollisionDemo ElasticCollisionDemo.cpp)
target_link_libraries(raiElasticCollisionDemo raiSim raiSimODE)

add_executable(raiURDFDemo URDFDemo.cpp)
target_link_libraries(raiURDFDemo raiSim raiSimODE)