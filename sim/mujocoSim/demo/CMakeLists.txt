add_executable(MjcSimDemo MjcSimDemo.cpp)
target_link_libraries(MjcSimDemo ${MjcSimLink})

add_executable(MjcAnymalDemo AnymalDemo.cpp)
target_link_libraries(MjcAnymalDemo ${MjcSimLink})

add_executable(MjcBuildingDemo BuildingDemo.cpp)
target_link_libraries(MjcBuildingDemo ${MjcSimLink})