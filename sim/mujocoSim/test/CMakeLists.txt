add_executable(mujocoSimulationTest SpeedTest.cpp)
target_link_libraries(mujocoSimulationTest ${MjcSimLink})

add_executable(mujocoMassTest MassTest.cpp)
target_link_libraries(mujocoMassTest ${MjcSimLink})

add_executable(mujocoForceTest ForceTest.cpp)
target_link_libraries(mujocoForceTest ${MjcSimLink})