add_executable(yamlTest yamlTest.cpp)
target_link_libraries(yamlTest ${YAML_CPP_LIBRARIES})

if(BENCHMARK_RAISIM AND raiSim_FOUND)
    add_executable(raisimTest raisimTest.cpp)
    target_link_libraries(raisimTest ${RAI_SIM_LINK})
endif()

if(BENCHMARK_BULLET AND Bullet_FOUND)
    add_executable(bulletTest bulletTest.cpp)
    target_link_libraries(bulletTest BulletCollision BulletDynamics LinearMath)
endif()

if(BENCHMARK_ODE AND ode_FOUND)
    add_executable(odeTest odeTest.cpp)
    target_link_libraries(odeTest ode)
endif()

if(BENCHMARK_MUJOCO)
    add_executable(mujocoTest mujocoTest.cpp)
    target_link_libraries(mujocoTest ${MuJoCoSimLink})
endif()