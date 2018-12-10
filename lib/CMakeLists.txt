#[[

This CMakeLists.txt is for installing dependencies.

"externalproject_add" will download each dependency into build directory, build and install.
The path convention is

<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/
<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/src
<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/include
<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/lib
<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/bin
...


If this is first build, build "ex_all" first to download all dependencies, build and install.
Then cmake will find each dependencies from the <CMAKE BUILD DIR>.

]]#


# cmake externalproject
include(ExternalProject)

# raigraphics
externalproject_add(ex_raigraphics
        GIT_REPOSITORY      "https://github.com/eastskykang/raigraphics.git"
        GIT_TAG             "master"
        CMAKE_ARGS
        "-DINSTALL_LIBS=ON"
        "-DBUILD_DEMOS=OFF"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )

# ode
externalproject_add(ex_ode
        GIT_REPOSITORY      "https://github.com/eastskykang/ode.git"
        GIT_TAG             "release/0.15.2"
        CMAKE_ARGS
        "-DBUILD_SHARED_LIBS=OFF"
        "-DODE_DOUBLE_PRECISION=ON"
        "-DODE_WITH_DEMOS=OFF"
        "-DODE_WITH_TESTS=OFF"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )

# bullet
externalproject_add(ex_bullet
        GIT_REPOSITORY      "https://github.com/eastskykang/bullet3.git"
        GIT_TAG             "release/2.87"
        CMAKE_ARGS
        "-DUSE_DOUBLE_PRECISION=ON"
        "-DBUILD_SHARED_LIBS=OFF"
        "-DBULLET2_MULTITHREADING=OFF"
        "-DBUILD_BULLET3=ON"
        "-DBUILD_PYBULLET=OFF"
        "-DBUILD_BULLET2_DEMOS=OFF"
        "-DBUILD_EXTRAS=ON"
        "-DBUILD_UNIT_TESTS=OFF"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )

# dart
externalproject_add(ex_dart
        GIT_REPOSITORY      "https://github.com/eastskykang/dart.git"
        GIT_TAG             "release/6.6.2"
        CMAKE_ARGS
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )

# mujoco
externalproject_add(ex_mujoco
        URL                 "https://www.roboti.us/download/mujoco200_linux.zip"
        CONFIGURE_COMMAND   ""
        BUILD_COMMAND       ""
        INSTALL_COMMAND     ""
        )


########################################################################################################################
# add ex_all as build target
message(STATUS "The project depends on the following external libraries")
message(STATUS "Build ex_all and cmake again if this is first build:")
message(STATUS "  ex_raigraphics")

list(APPEND ex_projects ex_raigraphics)

if(RAISIM)
    message(STATUS "  ex_raiSim")
    list(APPEND ex_projects ex_raisim)
endif(RAISIM)

if(BULLET)
    message(STATUS "  ex_bullet")
    list(APPEND ex_projects ex_bullet)
endif(BULLET)

if(ODE)
    message(STATUS "  ex_ode")
    list(APPEND ex_projects ex_ode)
endif(ODE)

if(MUJOCO)
    message(STATUS "  ex_mujoco")
    list(APPEND ex_projects ex_mujoco)
endif(MUJOCO)

if(DART)
    message(STATUS "  ex_dart")
    list(APPEND ex_projects ex_dart)
endif(DART)

add_custom_target(ex_all
        DEPENDS
        ${ex_projects}
        )