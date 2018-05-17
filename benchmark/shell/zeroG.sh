#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ANYMAL MOMENTUM BENCHMARK (ZERO G TEST)                                                                            ##
##                                                                                                                    ##
########################################################################################################################

echo "    ___    _   ____  ____  ______    __       _____   __________  ____        ______"
echo "   /   |  / | / /\ \/ /  |/  /   |  / /      /__  /  / ____/ __ \/ __ \      / ____/"
echo "  / /| | /  |/ /  \  / /|_/ / /| | / /         / /  / __/ / /_/ / / / /_____/ / __  "
echo " / ___ |/ /|  /   / / /  / / ___ |/ /___      / /__/ /___/ _, _/ /_/ /_____/ /_/ /  "
echo "/_/  |_/_/ |_/   /_/_/  /_/_/  |_/_____/     /____/_____/_/ |_|\____/      \____/   "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
dt_array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" )
csv_file=$( date +"%Y-%m-%d-%H:%M:%S" )

echo ""
echo "====================================================================="
for dt in ${dt_array[@]}
do
    # rai sim
    if [ "$test_rai" == 'ON' ]; then
        if [ "$RAISIM_ON" == "ON" ]; then
            timeout 600 ../sim/raiSim/benchmark/RaiAnymalMomentumBenchmark \
            --nogui --dt=$dt --csv=$csv_file #--log
        else
            echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
        fi
    fi

    # bullet sim
    if [ "$test_bt" == 'ON' ]; then
        if [ "$BTSIM_ON" == "ON" ]; then
            timeout 600 ../sim/bulletSim/benchmark/BtAnymalMomentumBenchmark \
            --nogui --dt=$dt --csv=$csv_file #--log
        else
            echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
        fi
    fi

    # ode sim
    if [ "$test_ode" == 'ON' ]; then
        if [ "$ODESIM_ON" == "ON" ] ; then
            for solver in std quick
            do
                timeout 600 ../sim/odeSim/benchmark/OdeAnymalMomentumBenchmark \
                --nogui --dt=$dt --solver=$solver --csv=$csv_file #--log
            done
        else
            echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
        fi
    fi

    # mujoco sim
    if [ "$test_mjc" == 'ON' ]; then
        if [ "$MJCSIM_ON" == "ON" ] ; then
            for solver in pgs cg newton
            do
                # note mujoco has no erp
                timeout 600 ../sim/mujocoSim/benchmark/MjcAnymalMomentumBenchmark \
                --nogui --dt=$dt --solver=$solver --csv=$csv_file #--log
            done
        else
            echo "mujocosim is not built. turn on BENCHMARK_MUJOCOSIM option in cmake"
        fi
    fi

    # dart sim
    if [ "$test_dart" == 'ON' ]; then
        if [ "$DARTSIM_ON" == "ON" ] ; then
            for solver in dantzig pgs
            do
                # note dart has no erp
                timeout 600 ../sim/dartSim/benchmark/DartAnymalMomentumBenchmark \
                --nogui --dt=$dt --solver=$solver --csv=$csv_file #--log
            done
        else
            echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
        fi
    fi
done

echo "Zero G test finished."