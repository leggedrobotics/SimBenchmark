add_executable(OdeStackDemo StackDemo.cpp)
target_link_libraries(OdeStackDemo ${ODESimLink})

add_executable(OdeAnymalDemo AnymalDemo.cpp)
target_link_libraries(OdeAnymalDemo ${ODESimLink})

add_executable(OdeMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(OdeMultiBodyDemo ${ODESimLink})

add_executable(OdeRobotArmDemo RobotArmDemo.cpp)
target_link_libraries(OdeRobotArmDemo ${ODESimLink})

add_executable(OdeCaplerDemo CaplerDemo.cpp)
target_link_libraries(OdeCaplerDemo ${ODESimLink})

# friction
#add_executable(frictionODESim frictionODESim.cpp)
#target_link_libraries(frictionODESim ${ODESimLink})