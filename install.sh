#!/usr/bin/env bash

ROOT_DIR = "$PWD"

raisim_flag=''
bullet_flag=''
ode_flag=''
mujoco_flag=''

# sim bench logo
echo "------------------------------------------------------------------------------"
echo "     _______. __  .___  ___. .______    _______ .__   __.   ______  __    __  "
echo "    /       ||  | |   \/   | |   _  \  |   ____||  \ |  |  /      ||  |  |  | "
echo "   |   (----'|  | |  \  /  | |  |_)  | |  |__   |   \|  | |  ,----'|  |__|  | "
echo "    \   \    |  | |  |\/|  | |   _  <  |   __|  |  . '  | |  |     |   __   | "
echo ".----)   |   |  | |  |  |  | |  |_)  | |  |____ |  |\   | |  '----.|  |  |  | "
echo "|_______/    |__| |__|  |__| |______/  |_______||__| \__|  \______||__|  |__| "
echo "                                                                              "
echo "------------------------------------------------------------------------------"

echo "Do you want to install and benchmark Bullet Physics [y/n]?"
read answer
echo "Do you want to install and benchmark ODE [y/n]?"
read answer
echo "Do you want to install and benchmark MuJoCo [y/n]?"
read answer
echo "Do you want to install and benchmark DART [y/n]?"

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
sudo make install -j4

# install yaml-cpp
echo "Installing yaml-cpp..."
cd $ROOT_DIR/lib
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
sudo rm -rf build
mkdir build && cd build
cmake ../
sudo make install -j4

# install raiSim (optional)
if [ "$raisim_flag" == 'true' ]; then
    echo "Installing raiSim..."
    echo "raiSim is currently only available for raiSim developers."
fi

# install bullet (optional)
if [ "$bullet_flag" == 'true' ]; then
    echo "Installing Bullet..."
    cd $ROOT_DIR/lib
    git clone https://github.com/bulletphysics/bullet3.git
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON ../
    sudo make install -j4
fi

# install ode (optional)
if [ "$ode_flag" == 'true' ]; then
    echo "Installing ODE... (0.15.2 version)"
    cd $ROOT_DIR/lib
    wget https://bitbucket.org/odedevs/ode/downloads/ode-0.15.2.tar.gz
fi

# install mujoco (optional)
if [ "mujoco_flag" == 'true' ]; then
    echo "Installing MuJoCo... (1.50 version)"
    wget https://www.roboti.us/download/mjpro150_linux.zip
fi

# bulid
mkdir $ROOT_DIR/build & cd $ROOT_DIR/build
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j4

# finished
cd $ROOT_DIR