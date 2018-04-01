add_executable(stackBulletSim stackBulletSim.cpp)
target_link_libraries(stackBulletSim ${BULLET_SIM_LINK})

add_executable(demoAnymalBulletSim demoAnymalBulletSim.cpp)
target_link_libraries(demoAnymalBulletSim ${BULLET_SIM_LINK})
