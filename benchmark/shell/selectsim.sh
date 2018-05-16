#!/usr/bin/env bash

# for get cmake options
source sim.sh

# read options
echo ""
echo "====================================================================="

while true; do
    read -p "Test RaiSim [y/n]? " yn
    case $yn in
        [Yy]* ) test_rai='ON'; break;;
        [Nn]* ) test_rai='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test Bullet [y/n]? " yn
    case $yn in
        [Yy]* ) test_bt='ON'; break;;
        [Nn]* ) test_bt='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test ODE [y/n]? " yn
    case $yn in
        [Yy]* ) test_ode='ON'; break;;
        [Nn]* ) test_ode='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test MuJoCo [y/n]? " yn
    case $yn in
        [Yy]* ) test_mjc='ON'; break;;
        [Nn]* ) test_mjc='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test Dart [y/n]? " yn
    case $yn in
        [Yy]* ) test_dart='ON'; break;;
        [Nn]* ) test_dart='OFF'; break;;
        * ) echo "Please answer y or n.";;
    esac
done


echo ""
echo "====================================================================="
if [ $test_rai == 'ON' ]
then
    if [ "$RAISIM_ON" == "ON" ]
    then
        echo "Test RaiSim."
    fi
fi
if [ $test_bt == 'ON' ]
then
    if [ "$BTSIM_ON" == "ON" ]
    then
        echo "Test Bullet."
    fi
fi
if [ $test_ode == 'ON' ]
then
    if [ "$ODESIM_ON" == "ON" ]
    then
        echo "Test ODE."
    fi
fi
if [ $test_mjc == 'ON' ]
then
    if [ "$MJCSIM_ON" == "ON" ]
    then
        echo "Test MuJoCo."
    fi
fi
if [ $test_dart == 'ON' ]
then
    if [ "$DARTSIM_ON" == "ON" ]
    then
        echo "Test Dart."
    fi
fi
