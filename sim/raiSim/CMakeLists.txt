# set raiSim backend
add_definitions(-DCOLLISION_BACKEND_ODE)        # ODE
#    add_definitions(-DCOLLISION_BACKEND_BULLET)     # BULLET

# raiSim
find_package(raiSim CONFIG REQUIRED)
include_directories(${RAI_SIM_INCLUDE_DIR})

add_subdirectory(test)
add_subdirectory(benchmark)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)