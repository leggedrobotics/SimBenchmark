#!/usr/bin/env bash

array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )

for dt in ${array[@]}
do
    # rai sim
    ./rollingRaiSim $dt

#    # bullet sim
#    for solver in seqImp nncg pgs dantzig #lemke
#    do
#        ./rollingBulletSim $dt $solver
#    done
#
#    # ode sim
#    for solver in std quick
#    do
#        ./rollingODESim $dt $solver
#    done
#
#    # mujoco sim
#    for solver in pgs cg newton
#    do
#        ./rollingMuJoCoSim $dt $solver
#    done
done