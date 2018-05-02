#!/usr/bin/env bash
max=10

### ---------------------------------------------
### ANYmal grounded
### ---------------------------------------------
#
## no slip off
#echo "---------------------------------------------"
#echo "ANYmal grounded (no-slip off)"
#echo "---------------------------------------------"
#
#echo "MuJoCo PGS"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=pgs --nogui --feedback=false
#done
#
#echo "MuJoCo CG"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=cg --nogui --feedback=false
#done
#
#echo "MuJoCo Newton"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=newton --nogui --feedback=false
#done
#
## no slip on
#echo "---------------------------------------------"
#echo "ANYmal grounded (no-slip on)"
#echo "---------------------------------------------"
#
#echo "MuJoCo PGS (no slip)"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=pgs --nogui --feedback=false --noslip
#done
#
#echo "MuJoCo CG (no slip)"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=cg --nogui --feedback=false --noslip
#done
#
#echo "MuJoCo Newton (no slip)"
#for (( i=1; i <= $max; ++i ))
#do
#    ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --solver=newton --nogui --feedback=false --noslip
#done

## ---------------------------------------------
## ANYmal grounded
## ---------------------------------------------

# no slip off
echo "---------------------------------------------"
echo "ANYmal grounded (no-slip off)"
echo "---------------------------------------------"

echo "MuJoCo PGS"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Ps 50
done

echo "MuJoCo CG"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Cs 50
done

echo "MuJoCo Newton"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Ns 50
done

# no slip on
echo "---------------------------------------------"
echo "ANYmal grounded (no-slip on)"
echo "---------------------------------------------"

echo "MuJoCo PGS (no slip)"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Pns 50
done

echo "MuJoCo CG (no slip)"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Cns 50
done

echo "MuJoCo Newton (no slip)"
for (( i=1; i <= $max; ++i ))
do
    ../sim/mujocoSim/test/MjcSimulationTest ../res/mujoco/ANYmal/robot.urdf Nns 50
done