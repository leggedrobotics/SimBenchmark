if(BENCHMARK_RAISIM AND raiSim_FOUND)
    # rolling
    add_executable(rollingRaiSim rollingRaiSim.cpp)
    target_link_libraries(rollingRaiSim ${RAI_SIM_LINK})

    # bounce
    add_executable(bounceRaiSim bounceRaiSim.cpp)
    target_link_libraries(bounceRaiSim ${RAI_SIM_LINK})

    # thousand
    add_executable(thousandRaiSim thousandRaiSim.cpp)
    target_link_libraries(thousandRaiSim ${RAI_SIM_LINK})
endif()

if(BENCHMARK_BULLET AND Bullet_FOUND)
    # rolling
    add_executable(rollingBulletSim rollingBulletSim.cpp)
    target_link_libraries(rollingBulletSim ${BulletSimLink})

    # bounce
    add_executable(bounceBulletSim bounceBulletSim.cpp)
    target_link_libraries(bounceBulletSim ${BulletSimLink})

    # thousand
    add_executable(thousandBulletSim thousandBulletSim.cpp)
    target_link_libraries(thousandBulletSim ${BulletSimLink})
endif()

if(BENCHMARK_ODE AND ode_FOUND)
    # rolling
    add_executable(rollingODESim rollingODESim.cpp)
    target_link_libraries(rollingODESim ${ODESimLink})

    # bounce
    add_executable(bounceODESim bounceODESim.cpp)
    target_link_libraries(bounceODESim ${ODESimLink})

    # thousand
    add_executable(thousandODESim thousandODESim.cpp)
    target_link_libraries(thousandODESim ${ODESimLink})
endif()

if(BENCHMARK_MUJOCO)
    # rolling
    add_executable(rollingMuJoCoSim rollingMuJoCoSim.cpp)
    target_link_libraries(rollingMuJoCoSim ${MuJoCoSimLink})

    # bouncing
    add_executable(bounceMuJoCoSim bounceMuJoCoSim.cpp)
    target_link_libraries(bounceMuJoCoSim ${MuJoCoSimLink})
endif()

## --------------------------- export files ---------------------------
configure_file(shell/rolling.sh.in ${CMAKE_CURRENT_BINARY_DIR}/rolling.sh @ONLY)
configure_file(shell/bounce.sh.in ${CMAKE_CURRENT_BINARY_DIR}/bounce.sh @ONLY)

file(COPY rolling.yaml
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})