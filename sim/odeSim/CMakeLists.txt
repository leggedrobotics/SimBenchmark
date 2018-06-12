#set(CMAKE_CXX_FLAGS_DEBUG
#        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -DODESIM_DEBUG")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# ode double precision
add_definitions(-DODE_DOUBLE_PRECISION)

include_directories(${BENCHMARK_LIB_DIR}/ode/include)
link_directories(${BENCHMARK_LIB_DIR}/ode)

# source
include_directories(src)

set(ODESim_SRCS
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
        src/object/ArticulatedSystem/OdeLinkJoint.hpp)

# library
add_library(OdeSimLib ${ODESim_SRCS})
target_link_libraries(OdeSimLib
        ode
        urdfdom_model
        )

# link
set(OdeSimLink
        OdeSimLib
        ${BENCHMARK_LINK}
        )

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