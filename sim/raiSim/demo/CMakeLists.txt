add_executable(RaiStackDemo StackDemo.cpp)
target_link_libraries(RaiStackDemo ${RAI_SIM_LINK})

add_executable(RaiAnymalDemo AnymalDemo.cpp)
target_link_libraries(RaiAnymalDemo ${RAI_SIM_LINK})

add_executable(RaiMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(RaiMultiBodyDemo ${RAI_SIM_LINK})

add_executable(RaiRobotArmDemo RobotArmDemo.cpp)
target_link_libraries(RaiRobotArmDemo ${RAI_SIM_LINK})

add_executable(RaiBuildingDemo BuildingDemo.cpp)
target_link_libraries(RaiBuildingDemo ${RAI_SIM_LINK})

add_executable(RaiElasticCollisionDemo ElasticCollisionDemo.cpp)
target_link_libraries(RaiElasticCollisionDemo ${RAI_SIM_LINK})

add_executable(collisionAnymalRaiSim collisionAnymalRaiSim.cpp)
target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

add_executable(freeDropAnymalRaiSim freeDropAnymalRaiSim.cpp)
target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})