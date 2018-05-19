#!/usr/bin/env bash

# for get cmake options
source sim.sh

# read options
echo ""
echo "====================================================================="

while true; do
    read -p "Test RaiSim [Y/n]? " yn
    case $yn in
        [Yy]* ) test_rai='ON'; break;;
        [Nn]* ) test_rai='OFF'; break;;
        []* ) test_rai='ON'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test Bullet [Y/n]? " yn
    case $yn in
        [Yy]* ) test_bt='ON'; break;;
        [Nn]* ) test_bt='OFF'; break;;
        []* ) test_rai='ON'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test ODE [Y/n]? " yn
    case $yn in
        [Yy]* ) test_ode='ON'; break;;
        [Nn]* ) test_ode='OFF'; break;;
        []* ) test_rai='ON'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test MuJoCo [Y/n]? " yn
    case $yn in
        [Yy]* ) test_mjc='ON'; break;;
        [Nn]* ) test_mjc='OFF'; break;;
        []* ) test_rai='ON'; break;;
        * ) echo "Please answer y or n.";;
    esac
done

while true; do
    read -p "Test Dart [Y/n]? " yn
    case $yn in
        [Yy]* ) test_dart='ON'; break;;
        [Nn]* ) test_dart='OFF'; break;;
        []* ) test_rai='ON'; break;;
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
