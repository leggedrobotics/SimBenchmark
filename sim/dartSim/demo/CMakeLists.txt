add_executable(DartZeroGravityDemo ZeroGravityDemo.cpp)
target_link_libraries(DartZeroGravityDemo ${DartSimLink})

add_executable(DartAnymalDemo AnymalDemo.cpp)
target_link_libraries(DartAnymalDemo ${DartSimLink})

add_executable(DartRigidBodyDemo RigidBodyDemo.cpp)
target_link_libraries(DartRigidBodyDemo ${DartSimLink})

add_executable(DartBuildingDemo BuildingDemo.cpp)
target_link_libraries(DartBuildingDemo ${DartSimLink})