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

# yaml-cpp
externalproject_add(ex_yamlcpp
        GIT_REPOSITORY      "https://github.com/jbeder/yaml-cpp.git"
        GIT_TAG             "yaml-cpp-0.6.2"
        CMAKE_ARGS
        "-DBUILD_SHARED_LIBS=OFF"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )


# raicommon
externalproject_add(ex_raicommon
        GIT_REPOSITORY      "https://github.com/eastskykang/raicommon.git"
        GIT_TAG             "master"
        CMAKE_ARGS
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )


# raigraphics
externalproject_add(ex_raigraphics
        GIT_REPOSITORY      "https://github.com/eastskykang/raigraphics.git"
        GIT_TAG             "master"
        CMAKE_ARGS
        "-DINSTALL_LIBS=ON"
        "-DBUILD_DEMOS=OFF"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        )
externalproject_get_property(ex_raigraphics INSTALL_DIR)
set(raigraphics_install_dir ${INSTALL_DIR})

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

# raisim (if user has proper authentication)
find_file(raiSim_authentication
        CMakeLists.txt
        PATH                "${CMAKE_SOURCE_DIR}/res/raiSim"
        )

if(RAISIM)
    if(EXISTS ${raiSim_authentication})
        message(STATUS "Found raiSim Git Repo authentication.")
        include(${raiSim_authentication})

        externalproject_add(ex_gtest
                GIT_REPOSITORY      "https://github.com/abseil/googletest.git"
                GIT_TAG             "release-1.8.1"
                CMAKE_ARGS
                "-DINSTALL_GTEST=ON"
                "-DBUILD_GMOCK=OFF"
                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
                )
        externalproject_get_property(ex_gtest INSTALL_DIR)
        set(gtest_install_dir "${INSTALL_DIR}")

        externalproject_add(ex_raicommon_for_raisim
                GIT_REPOSITORY      "https://donghokang@bitbucket.org/jhwangbo/raicommon.git"
                GIT_TAG             "master"
                CMAKE_ARGS
                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
                )
        externalproject_get_property(ex_raicommon_for_raisim INSTALL_DIR)
        set(raicommon_for_raisim_install_dir ${INSTALL_DIR})

        externalproject_add(ex_raigraphics_for_raisim
                GIT_REPOSITORY      "https://bitbucket.org/jhwangbo/raigraphics_opengl.git"
                GIT_TAG             "feature/staticlib"
                CMAKE_ARGS
                "-DBUILD_SHARED_LIBS=OFF"
                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
                )
        externalproject_get_property(ex_raigraphics_for_raisim INSTALL_DIR)
        set(raigraphics_for_raisim_install_dir ${INSTALL_DIR})

        LIST(APPEND raisim_find_path
                "${gtest_install_dir}"
                "${raigraphics_for_raisim_install_dir}"
                )

        externalproject_add(ex_raisim
                GIT_REPOSITORY      "https://${BITBUCKET_USERNAME}@${RAISIM_REPOSITORY}"
                GIT_TAG             "simbenchmark"
                CMAKE_ARGS
                "-DCOLLISION_BACKEND=ODE"
                "-DBUILD_TEST=OFF"
                "-DBUILD_DEMO=OFF"
                "-DCMAKE_FIND_ROOT_PATH=${raisim_find_path}"
                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
                DEPENDS
                "ex_raicommon_for_raisim"
                "ex_raigraphics_for_raisim"
                "ex_gtest"
                )
    else()
        message(WARNING "Cannot find raiSim authentication.")
    endif()
endif()



########################################################################################################################
# add ex_all as build target
message(STATUS "The project depends on the following external libraries")
message(STATUS "Build ex_all and cmake again if this is first build:")
message(STATUS "  ex_raigraphics")

# common libs
list(APPEND ex_projects ex_yamlcpp)
list(APPEND ex_projects ex_raicommon)
list(APPEND ex_projects ex_raigraphics)

if(RAISIM)
    message(STATUS "  ex_raiSim")
    list(APPEND ex_projects ex_gtest)
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