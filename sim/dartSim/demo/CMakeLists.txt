add_executable(dartZeroGravityDemo ZeroGravityDemo.cpp)
target_link_libraries(dartZeroGravityDemo ${DartSimLink})

add_executable(dartAnymalDemo AnymalDemo.cpp)
target_link_libraries(dartAnymalDemo ${DartSimLink})

add_executable(dartAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(dartAnymalEnergyDemo ${DartSimLink})

add_executable(dartAtlasDemo AtlasDemo.cpp)
target_link_libraries(dartAtlasDemo ${DartSimLink})

add_executable(dartRigidBodyDemo RigidBodyDemo.cpp)
target_link_libraries(dartRigidBodyDemo ${DartSimLink})

add_executable(dartBuildingDemo BuildingDemo.cpp)
target_link_libraries(dartBuildingDemo ${DartSimLink})

add_executable(dartStackDemo StackDemo.cpp)
target_link_libraries(dartStackDemo ${DartSimLink})