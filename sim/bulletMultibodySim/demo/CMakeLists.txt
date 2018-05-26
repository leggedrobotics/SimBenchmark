add_executable(BtMbAnymalDemo AnymalDemo.cpp)
target_link_libraries(BtMbAnymalDemo ${BtMbSimLink})

add_executable(BtMbURDFDemo URDFDemo.cpp)
target_link_libraries(BtMbURDFDemo ${BtMbSimLink})