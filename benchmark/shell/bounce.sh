#!/usr/bin/env bash

array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )

for dt in ${array[@]}
do
    # rai sim
    ./bounceRaiSim $dt

    # bullet sim
    for solver in seqImp nncg pgs dantzig lemke
    do
        ./bounceBulletSim $dt $solver
    done

    # ode sim
    for solver in std quick
    do
        ./bounceODESim $dt $solver
    done
done