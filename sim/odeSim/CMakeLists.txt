set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

# ode double precision
add_definitions(-DODE_DOUBLE_PRECISION)

find_package(ode CONFIG REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})

# source
include_directories(src)

set(ODESim_SRCS
        src/OdeWorld.hpp
        src/OdeWorld.cpp
        src/OdeWorld_RG.hpp
        src/OdeWorld_RG.cpp
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
add_library(ODESimLib ${ODESim_SRCS})
target_link_libraries(ODESimLib
        ode
        urdfdom_model
        )

# link
set(ODESimLink
        ODESimLib
        ${BENCHMARK_LINK}
        )

# sub-directories
add_subdirectory(demo)