message(STATUS "Build odeSim")

########################################################################################################################
# dependencies

# ode
externalproject_get_property(ex_ode INSTALL_DIR)
set(ode_include_dir "${INSTALL_DIR}/include")
set(ode_lib_dir "${INSTALL_DIR}/lib")

find_library(ode
        NO_DEFAULT_PATH
        NAMES       ode
        PATHS       ${ode_lib_dir})

if(NOT ode)
    message(WARNING "Cannot find ode libs. Stop build odeSim")
    return()
endif()

# raigraphics
externalproject_get_property(ex_raigraphics INSTALL_DIR)
#find_package(raigraphics
#        CONFIG
#        REQUIRED
#        NO_DEFAULT_PATH
#        PATHS           "${INSTALL_DIR}"
#        )


########################################################################################################################
# flags
#set(CMAKE_CXX_FLAGS_DEBUG
#        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -DODESIM_DEBUG")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# ode double precision
add_definitions(-DODE_DOUBLE_PRECISION)


########################################################################################################################
# source
set(odeSimSrc
        src/OdeWorld.hpp
        src/OdeWorld.cpp
        src/OdeSim.hpp
        src/OdeSim.cpp
        src/UserHandle.hpp
        src/object/OdeSingleBodyObject.hpp
        src/object/OdeSingleBodyObject.cpp
        src/object/OdeBox.hpp
        src/object/OdeBox.cpp
        src/object/OdeCheckerBoard.hpp
        src/object/OdeCheckerBoard.cpp
        src/object/OdeSphere.cpp
        src/object/OdeSphere.hpp
        src/object/OdeCapsule.hpp
        src/object/OdeCapsule.cpp
        src/object/OdeCylinder.cpp
        src/object/OdeCylinder.hpp
        src/object/OdeObject.hpp
        src/object/OdeObject.cpp
        src/object/ArticulatedSystem/OdeArticulatedSystem.cpp
        src/object/ArticulatedSystem/OdeArticulatedSystem.hpp
        src/object/ArticulatedSystem/OdeLinkJoint.hpp
        )


########################################################################################################################
# library
add_library(odeSim
        STATIC
        ${odeSimSrc}
        )

add_dependencies(odeSim
        ex_ode
        ex_raigraphics
        commonSim
        )

target_include_directories(odeSim
        PUBLIC
        "src"
        "${ode_include_dir}"
        "${raigraphics_INCLUDE_DIRS}"
        )

target_link_libraries(odeSim
        ${ode}
        commonSim
        urdfdom_model
        )


########################################################################################################################
# sub-directories
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

if(BUILD_DEMO)
    add_subdirectory(demo)
endif(BUILD_DEMO)

if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)