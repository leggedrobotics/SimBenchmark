#!/usr/bin/env bash

array=( "0.001" "0.01" "0.1" )

for dt in ${array[@]}
do
    echo "raiSim"
    ./rollingRaiSim $dt
    ./rollingBulletSim $dt
    ./rollingODESim $dt
    ./rollingMuJoCoSim $dt
done