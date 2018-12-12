#!/usr/bin/env bash

set -e

echo "SimBenchmark Build"
echo ""
echo "==========================================================================="
echo "Install dependencies..."
echo ""

# install apt packages
echo "Install apt packages..."

sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update

sudo apt install -y -qq g++-7
sudo apt install -y -qq \
libeigen3-dev \
libboost-all-dev \
libglew-dev \
libglm-dev \
libsdl2-dev \
libassimp-dev \
libsoil-dev \
libsdl2-ttf-dev \
liburdfdom-dev \
libgtest-dev \
google-perftools \
libgoogle-perftools-dev \
libpng-dev

# create build directory
echo "Creating build directory..."
rm -rf build
mkdir build && cd build

# cmake
echo "Configuring dependencies by cmake externalproject..."
cmake ../

# build
echo "Building dependencies (it takes minutes)..."
make ex_all -j4

echo "==========================================================================="
echo "Install SimBenchmark..."
echo ""

# cmake
echo "Configuring SimBenchmark by cmake..."
cmake ../

# build
echo "Building SimBenchmark..."
make -j4

echo "==========================================================================="
echo "SimBenchmark build done!"
echo ""

#
#
#SIMBENCHMARK_ROOT_DIR="$PWD"
#
#raisim_flag='OFF'
#bullet_flag='OFF'
#ode_flag='OFF'
#mujoco_flag='OFF'
#dart_flag='OFF'
#
## sim bench logo
#echo "   _____ ______  _______  _______   __________  ____  ______    ____  __ __"
#echo "  / ___//  _/  |/  / __ )/ ____/ | / / ____/ / / /  |/  /   |  / __ \/ //_/"
#echo "  \__ \ / // /|_/ / __  / __/ /  |/ / /   / /_/ / /|_/ / /| | / /_/ / ,<   "
#echo " ___/ // // /  / / /_/ / /___/ /|  / /___/ __  / /  / / ___ |/ _, _/ /| |  "
#echo "/____/___/_/  /_/_____/_____/_/ |_/\____/_/ /_/_/  /_/_/  |_/_/ |_/_/ |_|  "
#
#echo ""
#echo "==========================================================================="
#
## select sim engines
#while true; do
#    read -p "Do you want to install and benchmark raiSim [y/n]? " yn
#    case $yn in
#        [Yy]* ) raisim_flag='ON'; break;;
#        [Nn]* ) raisim_flag='OFF'; break;;
#        * ) echo "Please answer y or n.";;
#    esac
#done
#
#while true; do
#    read -p "Do you want to install and benchmark Bullet Physics [y/n]? " yn
#    case $yn in
#        [Yy]* ) bullet_flag='ON'; break;;
#        [Nn]* ) bullet_flag='OFF'; break;;
#        * ) echo "Please answer y or n.";;
#    esac
#done
#
#while true; do
#    read -p "Do you want to install and benchmark ODE [y/n]? " yn
#    case $yn in
#        [Yy]* ) ode_flag='ON'; break;;
#        [Nn]* ) ode_flag='OFF'; break;;
#        * ) echo "Please answer y or n.";;
#    esac
#done
#
#while true; do
#    read -p "Do you want to install and benchmark MuJoCo [y/n]? " yn
#    case $yn in
#        [Yy]* ) mujoco_flag='ON'; break;;
#        [Nn]* ) mujoco_flag='OFF'; break;;
#        * ) echo "Please answer y or n.";;
#    esac
#done
#
#while true; do
#    read -p "Do you want to install and benchmark DART [y/n]? " yn
#    case $yn in
#        [Yy]* ) dart_flag='ON'; break;;
#        [Nn]* ) dart_flag='OFF'; break;;
#        * ) echo "Please answer y or n.";;
#    esac
#done
#
#echo "==========================================================================="
#
## install build tools
#echo "Install build tools..."
#sudo add-apt-repository ppa:ubuntu-toolchain-r/test
#sudo apt update
#sudo apt install -y -qq g++-7
#
## install dependencies
#echo "Install dependencies..."
##sudo rm -rf $ROOT_DIR/lib/mjpro150
#sudo apt install -y -qq libeigen3-dev libboost-all-dev libglew-dev libglm-dev libsdl2-dev \
#libassimp-dev libsoil-dev libsdl2-ttf-dev liburdfdom-dev
#
## check if git is installed
#echo "Check if git is installed."
#if dpkg-query -W -f'${Status}' "git" 2>/dev/null | grep -q "ok installed"; then
#    echo "git is installed."
#else
#    echo "git is not installed. stop installing benchmark"
#    exit -1
#fi
#
## install raiSim (optional)
#if [ "$raisim_flag" == 'ON' ]; then
#    echo "Installing raiSim..."
#    echo "raiSim is currently only available for raiSim developers."
#fi
#
## install mujoco (optional)
#if [ "mujoco_flag" == 'ON' ]; then
#    echo "Installing MuJoCo... (1.50 version)"
#    cd $ROOT_DIR/lib
#    unzip mjpro150_linux.zip
#    wget https://www.roboti.us/download/mjpro150_linux.zip
#    rm mjpro150_linux.zip
#fi
#
## bulid
#mkdir $SIMBENCHMARK_ROOT_DIR/build & cd $SIMBENCHMARK_ROOT_DIR/build
#cmake -DCMAKE_BUILD_TYPE=Release ../
#make
#
## finished
#cd $SIMBENCHMARK_ROOT_DIR
#
#echo "==========================================================================="
#echo "SimBenchmark installed."
#echo "Put mjkey.txt file into lib/mjpro150"
