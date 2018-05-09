add_executable(OdeStackDemo StackDemo.cpp)
target_link_libraries(OdeStackDemo ${OdeSimLink})

add_executable(OdeAnymalDemo AnymalDemo.cpp)
target_link_libraries(OdeAnymalDemo ${OdeSimLink})

add_executable(OdeMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(OdeMultiBodyDemo ${OdeSimLink})

add_executable(OdeRobotArmDemo RobotArmDemo.cpp)
target_link_libraries(OdeRobotArmDemo ${OdeSimLink})

add_executable(OdeCaplerDemo CaplerDemo.cpp)
target_link_libraries(OdeCaplerDemo ${OdeSimLink})

add_executable(OdeBuildingDemo BuildingDemo.cpp)
target_link_libraries(OdeBuildingDemo ${OdeSimLink})

# friction
#add_executable(frictionODESim frictionODESim.cpp)
#target_link_libraries(frictionODESim ${ODESimLink})