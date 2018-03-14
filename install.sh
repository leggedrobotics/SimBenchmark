#!/usr/bin/env bash

## options
# -ODE
# -Bullet
# -raiSim

ROOT_DIR = "$PWD"

sudo rm -rf $ROOT_DIR/lib
mkdir $ROOT_DIR/lib

# check if git is installed
if dpkg-query -W -f'${Status}' "git" 2>/dev/null | grep -q "ok installed"; then
    echo "git is installed."
else
    echo "git is not installed. stop installing benchmark"
    exit
fi

# install raiCommon
echo "Installing raiCommon..."
cd $ROOT_DIR/lib
git clone git@bitbucket.org:jhwangbo/raicommon.git
cd raicommon
sudo rm -rf build
mkdir build && cd build
cmake ../
sudo make install

# install raiGraphicsOpengl
echo "Installing raiGraphicsOpengl..."
cd $ROOT_DIR/lib
git clone git@bitbucket.org:jhwangbo/raigraphics_opengl.git
cd raigraphics_opengl
sudo rm -rf build
mkdir build && cd build
cmake ../
sudo make install

# install yaml-cpp
echo "Installing yaml-cpp..."
cd $$ROOT_DIR/lib
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
sudo rm -rf build
mkdir build && cd build
cmake ../
sudo make install

# install raiSim (optional)
# install bullet (optional)
# install ode (optional)
