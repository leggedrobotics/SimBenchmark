message(STATUS "  odeSim Demos")


set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -DODESIM_DEBUG")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# ode double precision
add_definitions(-DODE_DOUBLE_PRECISION)

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

add_executable(odeBuildingDemo BuildingDemo.cpp)
target_link_libraries(odeBuildingDemo odeSim)

add_executable(odeURDFDemo URDFDemo.cpp)
target_link_libraries(odeURDFDemo odeSim)

