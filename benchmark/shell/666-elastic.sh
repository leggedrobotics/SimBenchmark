#!/usr/bin/env bash

DT_ARRAY=( "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.02" "0.03" "0.04" "0.05" "0.06" "0.07" "0.08" "0.09" "0.1" )

# options
raisim_flag='true'
bullet_flag='true'
ode_flag='true'
mujoco_flag='false'
dart_flag='true'

#while getopts 'rbom' flag; do
#  case "${flag}" in
#    r) raisim_flag='true'; echo "Test raisim" ;;
#    b) bullet_flag='true'; echo "Test bullet" ;;
#    o) ode_flag='true'; echo "Test ode" ;;
#    m) mujoco_flag='true'; echo "Test mujoco" ;;
#    *) error "Unexpected option ${flag}" ;;
#  esac
#done

# logo (slant)
echo "   _____ _____ _____    _______________________"
echo "  / ___// ___// ___/   /_  __/ ____/ ___/_  __/"
echo " / __ \/ __ \/ __ \     / / / __/  \__ \ / /   "
echo "/ /_/ / /_/ / /_/ /    / / / /___ ___/ // /    "
echo "\____/\____/\____/    /_/ /_____//____//_/     "

echo ""
echo "====================================================================="
echo "The log file is saved in data/thousand directory"
echo "====================================================================="

source sim.sh

# benchmark test
for dt in ${DT_ARRAY[@]}
do
    # rai sim
    if [ "$raisim_flag" == 'true' ]; then
        if [ "$RAISIM_ON" == "ON" ]; then
            for erpon in true false
            do
                timeout 600 ../sim/raiSim/benchmark/RaiThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --log --timer
            done
        else
            echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
        fi
    fi

    # bullet sim
    if [ "$bullet_flag" == 'true' ]; then
        if [ "$BTSIM_ON" == "ON" ]; then
            for solver in seqimp nncg pgs dantzig #lemke
            do
                for erpon in true false
                do
                    timeout 600 ../sim/bulletSim/benchmark/BtThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --log --timer
                done
            done
        else
            echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
        fi
    fi

    # ode sim
    if [ "$ode_flag" == 'true' ]; then
        if [ "$ODESIM_ON" == "ON" ] ; then
            for solver in std quick
            do
                for erpon in true false
                do
                    timeout 600 ../sim/odeSim/benchmark/OdeThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --log --timer
                done
            done
        else
            echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
        fi
    fi

    # mujoco sim
    if [ "$mujoco_flag" == 'true' ]; then
        if [ "$MJCSIM_ON" == "ON" ] ; then
            for solver in pgs cg newton
            do
                # note mujoco has no erp
                timeout 600 ../sim/mujocoSim/benchmark/MjcThousandBenchmark --nogui --dt=$dt --solver=$solver --log --timer
            done
        else
            echo "mujocosim is not built. turn on BENCHMARK_MUJOCOSIM option in cmake"
        fi
    fi

    # dart sim
    if [ "$dart_flag" == 'true' ]; then
        if [ "$DARTSIM_ON" == "ON" ] ; then
            for solver in dantzig pgs
            do
                # note dart has no erp
                timeout 600 ../sim/dartSim/benchmark/DartThousandBenchmark --nogui --dt=$dt --solver=$solver --log --timer
            done
        else
            echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
        fi
    fi
done
