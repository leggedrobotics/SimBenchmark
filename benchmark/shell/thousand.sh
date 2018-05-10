#!/usr/bin/env bash

DT_ARRAY=( "0.01" )

# options
raisim_installed="ON"
bulletsim_installed="ON"
odesim_installed="ON"
mujocosim_installed="ON"
dartsim_installed="ON"

raisim_flag='true'
bullet_flag='true'
ode_flag='true'
mujoco_flag='true'
dart_flag='false'


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
echo "  ________  ______  __  _______ ___    _   ______     _______________________"
echo " /_  __/ / / / __ \/ / / / ___//   |  / | / / __ \   /_  __/ ____/ ___/_  __/"
echo "  / / / /_/ / / / / / / /\__ \/ /| | /  |/ / / / /    / / / __/  \__ \ / /   "
echo " / / / __  / /_/ / /_/ /___/ / ___ |/ /|  / /_/ /    / / / /___ ___/ // /    "
echo "/_/ /_/ /_/\____/\____//____/_/  |_/_/ |_/_____/    /_/ /_____//____//_/     "

echo ""
echo "====================================================================="
echo "The log file is saved in data/thousand directory"
echo "====================================================================="

# benchmark test
for dt in ${DT_ARRAY[@]}
do
    # rai sim
    if [ "$raisim_flag" == 'true' ]; then
        if [ "$raisim_installed" == "ON" ]; then
            for erpon in true false
            do
                timeout 600 ../sim/raiSim/benchmark/RaiThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --log --timer
#                timeout 600 ../sim/raiSim/benchmark/RaiThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --timer
            done
        else
            echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
        fi
    fi

    # bullet sim
    if [ "$bullet_flag" == 'true' ]; then
        if [ "$bulletsim_installed" == "ON" ]; then
            for solver in seqimp nncg pgs dantzig #lemke
            do
                for erpon in true false
                do
                    timeout 600 ../sim/bulletSim/benchmark/BtThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --log --timer
#                    timeout 600 ../sim/bulletSim/benchmark/BtThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --timer
                done
            done
        else
            echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
        fi
    fi

    # ode sim
    if [ "$ode_flag" == 'true' ]; then
        if [ "$odesim_installed" == "ON" ] ; then
            for solver in std quick
            do
                for erpon in true false
                do
                    timeout 600 ../sim/odeSim/benchmark/OdeThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --log --timer
#                    timeout 600 ../sim/odeSim/benchmark/OdeThousandBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --timer
                done
            done
        else
            echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
        fi
    fi

    # mujoco sim
    if [ "$mujoco_flag" == 'true' ]; then
        if [ "$mujocosim_installed" == "ON" ] ; then
            for solver in pgs cg newton
            do
                for erpon in true false
                do
                    # note mujoco has no erp
                    timeout 600 ../sim/mujocoSim/benchmark/MjcThousandBenchmark --nogui --dt=$dt --solver=$solver --log --timer
#                    timeout 600 ../sim/mujocoSim/benchmark/MjcThousandBenchmark --nogui --dt=$dt --solver=$solver --timer
                done
            done
        else
            echo "mujocosim is not built. turn on BENCHMARK_MUJOCOSIM option in cmake"
        fi
    fi

    # dart sim
    if [ "$dart_flag" == 'true' ]; then
        if [ "$dartsim_installed" == "ON" ] ; then
            for solver in dantzig pgs
            do
                for erpon in true false
                do
                    # note dart has no erp
                    timeout 600 ../sim/dartSim/benchmark/DartThousandBenchmark --nogui --dt=$dt --solver=$solver --log --timer
#                    timeout 600 ../sim/dartSim/benchmark/DartThousandBenchmark --nogui --dt=$dt --solver=$solver --timer
                done
            done
        else
            echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
        fi
    fi
done
