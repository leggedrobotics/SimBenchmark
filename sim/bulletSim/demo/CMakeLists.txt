add_executable(BtStackDemo StackDemo.cpp)
target_link_libraries(BtStackDemo ${BtSimLink})

add_executable(BtAnymalDemo AnymalDemo.cpp)
target_link_libraries(BtAnymalDemo ${BtSimLink})

add_executable(BtAtlasDemo AtlasDemo.cpp)
target_link_libraries(BtAtlasDemo ${BtSimLink})

add_executable(BtBuildingDemo BuildingDemo.cpp)
target_link_libraries(BtBuildingDemo ${BtSimLink})

add_executable(BtURDFDemo URDFDemo.cpp)
target_link_libraries(BtURDFDemo ${BtSimLink})
