#!/usr/bin/env bash

max=10

# bullet
echo "Bullet"
for (( i=1; i <= $max; ++i ))
do
    ../sim/bulletSim/demo/BtAnymalDemo
done

# rai
echo "Rai"
for (( i=1; i <= $max; ++i ))
do
    ../sim/raiSim/demo/RaiAnymalDemo
done

# mujoco
echo "MuJoCo PGS"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/demo/MjcAnymalDemo --solver=pgs
done

echo "MuJoCo CG"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/demo/MjcAnymalDemo --solver=cg
done

echo "MuJoCo Newton"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/demo/MjcAnymalDemo --solver=newton
done