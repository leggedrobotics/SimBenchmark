#!/usr/bin/env bash

ROOT_DIR = "$PWD"

raisim_flag='OFF'
bullet_flag='OFF'
ode_flag='OFF'
mujoco_flag='OFF'
dart_flag='OFF'

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

# select sim engines
while true; do
    read -p "Do you want to install and benchmark raiSim [y/n]? " yn
    case $yn in
        [Yy]* ) raisim_flag='ON'; break;;
        [Nn]* ) raisim_flag='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Do you want to install and benchmark Bullet Physics [y/n]? " yn
    case $yn in
        [Yy]* ) bullet_flag='ON'; break;;
        [Nn]* ) bullet_flag='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Do you want to install and benchmark ODE [y/n]? " yn
    case $yn in
        [Yy]* ) ode_flag='ON'; break;;
        [Nn]* ) ode_flag='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Do you want to install and benchmark MuJoCo [y/n]? " yn
    case $yn in
        [Yy]* ) mujoco_flag='ON'; break;;
        [Nn]* ) mujoco_flag='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Do you want to install and benchmark DART [y/n]? " yn
    case $yn in
        [Yy]* ) dart_flag='ON'; break;;
        [Nn]* ) dart_flag='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

# install dependencies
echo "Install dependencies..."
#sudo rm -rf $ROOT_DIR/lib/mjpro150

# check if git is installed
echo "Check if git is installed."
if dpkg-query -W -f'${Status}' "git" 2>/dev/null | grep -q "ok installed"; then
    echo "git is installed."
else
    echo "git is not installed. stop installing benchmark"
    exit -1
fi

# install raiSim (optional)
if [ "$raisim_flag" == 'ON' ]; thenf
    echo "Installing raiSim..."
    echo "raiSim is currently only available for raiSim developers."
fi

# install bullet (optional)
if [ "$bullet_flag" == 'ON' ]; then
    echo "Installing Bullet..."
    cd $ROOT_DIR/lib
    git clone https://github.com/bulletphysics/bullet3.git
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON ../
    sudo make install -j4
fi

# install ode (optional)
if [ "$ode_flag" == 'ON' ]; then
    echo "Installing ODE... (0.15.2 version)"
    cd $ROOT_DIR/lib
    wget https://bitbucket.org/odedevs/ode/downloads/ode-0.15.2.tar.gz
    tar -xvf ode-0.15.2.tar.gz
    cd ode-0.15.2
    mkdir cmake-build && cd cmake-build
    cmake -DCMAKE_BUILD_TYPE=Release -DODE_WITH_LIBCCD=ON ../
    sudo make install -j4
fi

# install mujoco (optional)
if [ "mujoco_flag" == 'ON' ]; then
    echo "Installing MuJoCo... (1.50 version)"
    cd $ROOT_DIR/lib
    unzip mjpro150_linux.zip
    wget https://www.roboti.us/download/mjpro150_linux.zip
    rm mjpro150_linux.zip
fi

# bulid
mkdir $ROOT_DIR/build & cd $ROOT_DIR/build
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j4

# finished
cd $ROOT_DIR

echo "Put mjkey.txt file into lib/mjpro150"
