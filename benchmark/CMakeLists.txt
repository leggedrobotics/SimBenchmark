file(COPY shell/anymal-pd.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/rolling.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/kapla.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/666.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/bouncing.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/666-elastic.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/anymal-momentum.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/anymal-energy.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY shell/selectsim.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(sim.sh.in ${CMAKE_CURRENT_BINARY_DIR}/sim.sh @ONLY)


#if(BENCHMARK_BULLET)
#    # rolling-benchmark
#    add_executable(rollingBulletSim rollingBulletSim.cpp rolling-benchmark.hpp)
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
#    # rolling-benchmark
#    add_executable(rollingODESim rolling-benchmark.hpp)
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
#    # rolling-benchmark
#    add_executable(rollingMuJoCoSim rollingMuJoCoSim.cpp rolling-benchmark.hpp)
#    target_link_libraries(rollingMuJoCoSim ${MuJoCoSimLink})
#
#    # bouncing
#    add_executable(bounceMuJoCoSim bounceMuJoCoSim.cpp)
#    target_link_libraries(bounceMuJoCoSim ${MuJoCoSimLink})
#endif()
#
### --------------------------- export files ---------------------------
#configure_file(shell/rolling-benchmark.sh.in ${CMAKE_CURRENT_BINARY_DIR}/rolling-benchmark.sh @ONLY)
#configure_file(shell/bounce.sh.in ${CMAKE_CURRENT_BINARY_DIR}/bounce.sh @ONLY)
#
## mujoco model scripts
#file(COPY mujoco
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#
## parameter yaml
#file(COPY rolling-benchmark.yaml
#        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})