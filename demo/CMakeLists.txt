if(BENCHMARK_RAISIM AND raiSim_FOUND)
    # stack
    add_executable(stackRaiSim stackRaiSim.cpp)
    target_link_libraries(stackRaiSim ${RAI_SIM_LINK})

    # anymal
    add_executable(anymalRaiSim anymalRaiSim.cpp)
    target_link_libraries(anymalRaiSim ${RAI_SIM_LINK})

    # collision anymal
    add_executable(collisionAnymalRaiSim collisionAnymalRaiSim.cpp)
    target_link_libraries(collisionAnymalRaiSim ${RAI_SIM_LINK})

    # free drop anymal
    add_executable(freeDropAnymalRaiSim freeDropAnymalRaiSim.cpp)
    target_link_libraries(freeDropAnymalRaiSim ${RAI_SIM_LINK})
endif()

if(BENCHMARK_BULLET AND Bullet_FOUND)
    # stack
    add_executable(stackBulletSim stackBulletSim.cpp)
    target_link_libraries(stackBulletSim ${BulletSimLink})
endif()

if(BENCHMARK_ODE AND ode_FOUND)
    # stack
    add_executable(stackODESim stackODESim.cpp)
    target_link_libraries(stackODESim ${ODESimLink})

    # friction
    add_executable(frictionODESim frictionODESim.cpp)
    target_link_libraries(frictionODESim ${ODESimLink})
endif()