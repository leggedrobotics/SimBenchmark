add_executable(MjcSimulationTest SpeedTest.cpp)
target_link_libraries(MjcSimulationTest ${MjcSimLink})

add_executable(MjcMassTest MassTest.cpp)
target_link_libraries(MjcMassTest ${MjcSimLink})

add_executable(MjcForceTest ForceTest.cpp)
target_link_libraries(MjcForceTest ${MjcSimLink})