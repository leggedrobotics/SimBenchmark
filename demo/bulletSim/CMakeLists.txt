message(STATUS "  bulletSim Demos")

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native -march=native")

# bullet double precision
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)



add_executable(bulletAnymalDemo AnymalDemo.cpp)
target_link_libraries(bulletAnymalDemo bulletSim)

add_executable(bulletAnymalMomentumDemo AnymalMomentumDemo.cpp)
target_link_libraries(bulletAnymalMomentumDemo bulletSim)

add_executable(bulletBallMomentumDemo BallMomentumDemo.cpp)
target_link_libraries(bulletBallMomentumDemo bulletSim)

add_executable(bulletAnymalEnergyDemo AnymalEnergyDemo.cpp)
target_link_libraries(bulletAnymalEnergyDemo bulletSim)

add_executable(bulletBallEnergyDemo BallEnergyDemo.cpp)
target_link_libraries(bulletBallEnergyDemo bulletSim)

add_executable(bulletURDFDemo URDFDemo.cpp)
target_link_libraries(bulletURDFDemo bulletSim)