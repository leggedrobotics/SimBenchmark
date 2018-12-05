add_executable(odeDemoStack StackDemo.cpp)
target_link_libraries(odeDemoStack odeSim)

add_executable(odeAnymalDemo AnymalDemo.cpp)
target_link_libraries(odeAnymalDemo odeSim)

add_executable(odeAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(odeAnymalEnergyDemo odeSim)

add_executable(odeAnymalZeroGravityDemo AnymalZeroGravityDemo.cpp)
target_link_libraries(odeAnymalZeroGravityDemo odeSim)

add_executable(odeMultiBodyDemo MultiBodyDemo.cpp)
target_link_libraries(odeMultiBodyDemo odeSim)

add_executable(odeRobotArmDemo RobotArmDemo.cpp)
target_link_libraries(odeRobotArmDemo odeSim)

add_executable(odeCaplerDemo CaplerDemo.cpp)
target_link_libraries(odeCaplerDemo odeSim)

add_executable(odeBuildingDemo BuildingDemo.cpp)
target_link_libraries(odeBuildingDemo odeSim)

add_executable(odeURDFDemo URDFDemo.cpp)
target_link_libraries(odeURDFDemo odeSim)


# friction
#add_executable(frictionODESim frictionODESim.cpp)
#target_link_libraries(frictionODESim ${ODESimLink})