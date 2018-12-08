# This CMakeLists.txt is for installing dependencies
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