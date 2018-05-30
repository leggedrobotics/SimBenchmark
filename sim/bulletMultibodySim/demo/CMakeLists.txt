add_executable(BtMbAnymalDemo AnymalDemo.cpp)
target_link_libraries(BtMbAnymalDemo ${BtMbSimLink})

add_executable(BtMbAnymalMomentumDemo AnymalMomentumDemo.cpp)
target_link_libraries(BtMbAnymalMomentumDemo ${BtMbSimLink})

add_executable(BtMbBallMomentumDemo BallMomentumDemo.cpp)
target_link_libraries(BtMbBallMomentumDemo ${BtMbSimLink})

add_executable(BtMbAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(BtMbAnymalEnergyDemo ${BtMbSimLink})

add_executable(BtMbBallEnergyDemo BallEnergyDemo.cpp)
target_link_libraries(BtMbBallEnergyDemo ${BtMbSimLink})

add_executable(BtMbURDFDemo URDFDemo.cpp)
target_link_libraries(BtMbURDFDemo ${BtMbSimLink})