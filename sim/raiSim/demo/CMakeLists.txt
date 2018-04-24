add_executable(stackRaiSim stackRaiSim.cpp)
target_link_libraries(stackRaiSim ${RAI_SIM_LINK})

add_executable(RaiAnymalDemo AnymalDemo.cpp)
target_link_libraries(RaiAnymalDemo ${RAI_SIM_LINK})

add_executable(RaiMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(RaiMultiBodyDemo ${RAI_SIM_LINK})

add_executable(Rai2DRobotArmDemo 2DRobotArmDemo.cpp)
target_link_libraries(Rai2DRobotArmDemo ${RAI_SIM_LINK})

add_executable(collisionAnymalRaiSim collisionAnymalRaiSim.cpp)
target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

add_executable(freeDropAnymalRaiSim freeDropAnymalRaiSim.cpp)
target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})