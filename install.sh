#!/usr/bin/env bash

## options
# -ODE
# -Bullet
# -raiSim

ROOT_DIR = "$PWD"

raisim_flag=''
bullet_flag=''
ode_flag=''
mujoco_flag=''

while getopts 'rbom' flag; do
  case "${flag}" in
    r) raisim_flag='true'; echo "Install raisim" ;;
    b) bullet_flag='true'; echo "Install Bullet" ;;
    o) ode_flag='true'; echo "Install ODE" ;;
    m) mujoco_flag='true'; echo "Install MuJoCo" ;;
    *) error "Unexpected option ${flag}" ;;
  esac
done

sudo rm -rf $ROOT_DIR/lib/*

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
cd $ROOT_DIR/lib
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
sudo rm -rf build
mkdir build && cd build
cmake ../
sudo make install

# install raiSim (optional)
if [ "$raisim_flag" == 'true' ]; then
    echo "Installing raiSim..."
fi

# install bullet (optional)
if [ "$bullet_flag" == 'true' ]; then
    echo "Installing Bullet..."
    cd $ROOT_DIR/lib
    git clone https://github.com/bulletphysics/bullet3.git
fi

# install ode (optional)
if [ "$ode_flag" == 'true' ]; then
    echo "Installing ODE..."
fi