#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ANYMAL MOMENTUM BENCHMARK (ZERO G TEST)                                                                            ##
##                                                                                                                    ##
########################################################################################################################

echo "    ___    _   ____  ____  ______    __       __  _______  __  __________   __________  ____  ___"
echo "   /   |  / | / /\ \/ /  |/  /   |  / /      /  |/  / __ \/  |/  / ____/ | / /_  __/ / / /  |/  /"
echo "  / /| | /  |/ /  \  / /|_/ / /| | / /      / /|_/ / / / / /|_/ / __/ /  |/ / / / / / / / /|_/ / "
echo " / ___ |/ /|  /   / / /  / / ___ |/ /___   / /  / / /_/ / /  / / /___/ /|  / / / / /_/ / /  / /  "
echo "/_/  |_/_/ |_/   /_/_/  /_/_/  |_/_____/  /_/  /_/\____/_/  /_/_____/_/ |_/ /_/  \____/_/  /_/   "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
dt_array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" )
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

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
            timeout 600 ../sim/bulletMultibodySim/benchmark/BtMbAnymalMomentumBenchmark \
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
                for integrator in euler rk4
                do
                # note mujoco has no erp
                timeout 600 ../sim/mujocoSim/benchmark/MjcAnymalMomentumBenchmark \
                --nogui --dt=$dt --solver=$solver --csv=$csv_file --integrator=$integrator #--log
                done
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

echo "Zero G test is finished."