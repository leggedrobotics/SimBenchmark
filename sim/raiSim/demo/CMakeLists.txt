# stack
add_executable(stackRaiSim stackRaiSim.cpp)
target_link_libraries(stackRaiSim ${RAI_SIM_LINK})

# anymal
add_executable(RaiAnymalDemo AnymalDemo.cpp)
target_link_libraries(RaiAnymalDemo ${RAI_SIM_LINK})

# collision anymal
add_executable(collisionAnymalRaiSim collisionAnymalRaiSim.cpp)
target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

# free drop anymal
add_executable(freeDropAnymalRaiSim freeDropAnymalRaiSim.cpp)
target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})