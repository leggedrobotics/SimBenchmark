#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ANYMAL BENCHMARK                                                                                                   ##
##                                                                                                                    ##
########################################################################################################################

# the number of test (for num_row <= 3)
num_test=10

# the number of anymals
num_row_rai=( 1 2 3 4 5 7 10 15 )
num_row_mjc=( 1 2 3 4 5 7 10 15 )
num_row_ode=( 1 2 3 4 5 7 10 15 )
num_row_bt=( 1 2 3 4 )
num_row_dart=( 1 2 3 4 5 7 10 15 )

# feedback (PD control)
feedback=true

# logo (slant)
echo "    ___    _   ____  ____  ______    __       _______________________"
echo "   /   |  / | / /\ \/ /  |/  /   |  / /      /_  __/ ____/ ___/_  __/"
echo "  / /| | /  |/ /  \  / /|_/ / /| | / /        / / / __/  \__ \ / /   "
echo " / ___ |/ /|  /   / / /  / / ___ |/ /___     / / / /___ ___/ // /    "
echo "/_/  |_/_/ |_/   /_/_/  /_/_/  |_/_____/    /_/ /_____//____//_/     "

echo ""
echo "====================================================================="
echo "The log file is saved in data/anymal-XXX directory"

source sim.sh

# RAI
echo "====================================================================="
echo "RAI"
for num_row in ${num_row_rai[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/raiSim/benchmark/RaiAnymalBenchmark --nogui --feedback=$feedback --row=$num_row
        done
    else
        ../sim/raiSim/benchmark/RaiAnymalBenchmark --nogui --feedback=$feedback --row=$num_row
    fi
done

# BULLET
echo "====================================================================="
echo "BULLET (MULTIBODY SOLVER)"
for num_row in ${num_row_bt[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/bulletSim/benchmark/BtAnymalBenchmark --nogui --feedback=$feedback --row=$num_row
        done
    else
        ../sim/bulletSim/benchmark/BtAnymalBenchmark --nogui --feedback=$feedback --row=$num_row
    fi
done

# DART
echo "====================================================================="
echo "DART (DANTZIG - BULLET)"
for num_row in ${num_row_dart[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=dantzig --detector=bullet
        done
    else
        ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=dantzig --detector=bullet
    fi
done

echo "====================================================================="
echo "DART (DANTZIG - ODE)"
for num_row in ${num_row_dart[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=dantzig --detector=ode
        done
    else
        ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=dantzig --detector=ode
    fi
done

echo "====================================================================="
echo "DART (PGS - BULLET)"
for num_row in ${num_row_dart[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --detector=bullet
        done
    else
        ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --detector=bullet
    fi
done

echo "====================================================================="
echo "DART (PGS - ODE)"
for num_row in ${num_row_dart[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --detector=ode
        done
    else
        ../sim/dartSim/benchmark/DartAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --detector=ode
    fi
done

# MUJOCO
echo "====================================================================="
echo "MUJOCO (PGS)"
for num_row in ${num_row_mjc[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --noslip
        done
    else
        ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=pgs --noslip
    fi
done

echo "====================================================================="
echo "MUJOCO (CG)"
for num_row in ${num_row_mjc[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=cg --noslip
        done
    else
        ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=cg --noslip
    fi
done

echo "====================================================================="
echo "MUJOCO (NEWTON)"
for num_row in ${num_row_mjc[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=newton --noslip
        done
    else
        ../sim/mujocoSim/benchmark/MjcAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=newton --noslip
    fi
done

# ODE
echo "====================================================================="
echo "ODE (STANDARD SOLVER)"
for num_row in ${num_row_ode[@]}
do
    if [[ $num_row -le 3 ]]
    then
        for (( i=1; i <= $num_test; ++i ))
        do
            ../sim/odeSim/benchmark/OdeAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=std
        done
    else
        ../sim/odeSim/benchmark/OdeAnymalBenchmark --nogui --feedback=$feedback --row=$num_row --solver=std
    fi
done
