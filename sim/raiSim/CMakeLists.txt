# raiSim
find_package(raiSim CONFIG REQUIRED)
include_directories(${RAI_SIM_INCLUDE_DIR})

# set raiSim backend
add_definitions(-DCOLLISION_BACKEND=ODE)        # ODE
#    add_definitions(-DCOLLISION_BACKEND=BULLET)     # BULLET

if(BUILD_TEST)
add_subdirectory(test)
endif(BUILD_TEST)
