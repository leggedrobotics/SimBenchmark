#!/usr/bin/env bash

array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )

for dt in ${array[@]}
do
    echo "raiSim"
    ./rollingRaiSim $dt
    ./rollingBulletSim $dt
    ./rollingODESim $dt
    ./rollingMuJoCoSim $dt
done