add_executable(yamlTest testYamlCpp.cpp)
target_link_libraries(yamlTest ${YAML_CPP_LIBRARIES})

if(BENCHMARK_RAISIM AND raiSim_FOUND)
    add_executable(testRaisim testRaisim.cpp)
    target_link_libraries(testRaisim ${RAI_SIM_LINK})
endif()

if(BENCHMARK_BULLET AND Bullet_FOUND)
    add_executable(testBullet testBullet.cpp)
    target_link_libraries(testBullet BulletCollision BulletDynamics LinearMath)
endif()

if(BENCHMARK_ODE AND ode_FOUND)
    add_executable(testODE testODE.cpp)
    target_link_libraries(testODE ode)
endif()

if(BENCHMARK_MUJOCO)
    add_executable(testMujoco testMujoco.cpp)
    target_link_libraries(testMujoco ${MuJoCoSimLink})
endif()

if(BENCHMARK_DART)
    add_executable(testDart testDart.cpp)
    target_link_libraries(testDart ${DartSimLink})
endif()