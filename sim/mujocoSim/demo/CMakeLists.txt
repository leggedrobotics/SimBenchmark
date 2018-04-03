add_executable(MjcSimDemo MjcSimDemo.cpp)
target_link_libraries(MjcSimDemo ${MUJOCO_SIM_LINK})

add_executable(MjcAnymalDemo AnymalDemo.cpp)
target_link_libraries(MjcAnymalDemo ${MUJOCO_SIM_LINK})