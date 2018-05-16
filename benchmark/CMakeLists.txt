file(COPY shell/anymal.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/rolling.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/building.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/666.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/bouncing.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/666-elastic.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/zeroG.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/selectsim.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(sim.sh.in ${CMAKE_CURRENT_BINARY_DIR}/sim.sh @ONLY)


#if(BENCHMARK_BULLET)
#    # rolling
#    add_executable(rollingBulletSim rollingBulletSim.cpp rolling.hpp)
#    target_link_libraries(rollingBulletSim ${BulletSimLink})
#
#    # bounce
#    add_executable(bounceBulletSim bounceBulletSim.cpp)
#    target_link_libraries(bounceBulletSim ${BulletSimLink})
#
#    # thousand
#    add_executable(thousandBulletSim thousandBulletSim.cpp)
#    target_link_libraries(thousandBulletSim ${BulletSimLink})
#endif()
#
#if(BENCHMARK_ODE)
#    # rolling
#    add_executable(rollingODESim rolling.hpp)
#    target_link_libraries(rollingODESim ${ODESimLink})
#
#    # bounce
#    add_executable(bounceODESim bounceODESim.cpp)
#    target_link_libraries(bounceODESim ${ODESimLink})
#
#    # thousand
#    add_executable(thousandODESim thousandODESim.cpp)
#    target_link_libraries(thousandODESim ${ODESimLink})
#endif()
#
#if(BENCHMARK_MUJOCO)
#    # rolling
#    add_executable(rollingMuJoCoSim rollingMuJoCoSim.cpp rolling.hpp)
#    target_link_libraries(rollingMuJoCoSim ${MuJoCoSimLink})
#
#    # bouncing
#    add_executable(bounceMuJoCoSim bounceMuJoCoSim.cpp)
#    target_link_libraries(bounceMuJoCoSim ${MuJoCoSimLink})
#endif()
#
### --------------------------- export files ---------------------------
#configure_file(shell/rolling.sh.in ${CMAKE_CURRENT_BINARY_DIR}/rolling.sh @ONLY)
#configure_file(shell/bounce.sh.in ${CMAKE_CURRENT_BINARY_DIR}/bounce.sh @ONLY)
#
## mujoco model scripts
#file(COPY mujoco
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#
## parameter yaml
#file(COPY rolling.yaml
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})