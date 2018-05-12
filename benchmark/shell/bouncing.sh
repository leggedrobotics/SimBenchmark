#!/usr/bin/env bash


DT_ARRAY=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )

# options
raisim_installed="ON"
bulletsim_installed="ON"
odesim_installed="ON"
mujocosim_installed="ON"
dartsim_installed="ON"

raisim_flag='true'
bullet_flag='true'
ode_flag='true'
mujoco_flag='false'     # note that mujoco cannot simulate restitutional effect
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
echo "    ____  ____  __  ___   _____________   ________   _______________________"
echo "   / __ )/ __ \/ / / / | / / ____/  _/ | / / ____/  /_  __/ ____/ ___/_  __/"
echo "  / __  / / / / / / /  |/ / /    / //  |/ / / __     / / / __/  \__ \ / /   "
echo " / /_/ / /_/ / /_/ / /|  / /____/ // /|  / /_/ /    / / / /___ ___/ // /    "
echo "/_____/\____/\____/_/ |_/\____/___/_/ |_/\____/    /_/ /_____//____//_/     "

echo ""
echo "====================================================================="
echo "The log file is saved in data/rolling directory"
echo "====================================================================="

# benchmark test
for dt in ${DT_ARRAY[@]}
do
    for e in 1.0
    do
        # rai sim
        if [ "$raisim_flag" == 'true' ]; then
            if [ "$raisim_installed" == "ON" ]; then
                for erpon in true false
                do
                    timeout 600 ../sim/raiSim/benchmark/RaiRollingBenchmark --nogui --erp-on=$erpon --dt=$dt --e=$forcedir
                done
            else
                echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
            fi
        fi

        # bullet sim
        if [ "$bullet_flag" == 'true' ]; then
            if [ "$bulletsim_installed" == "ON" ]; then
                for solver in seqimp nncg pgs dantzig lemke
                do
                    for erpon in true false
                    do
                        timeout 600 ../sim/bulletSim/benchmark/BtRollingBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --e=$forcedir
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
                        timeout 600 ../sim/odeSim/benchmark/OdeRollingBenchmark --nogui --erp-on=$erpon --dt=$dt --solver=$solver --e=$forcedir
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
                        timeout 600 ../sim/mujocoSim/benchmark/MjcRollingBenchmark --nogui --dt=$dt --solver=$solver --e=$forcedir
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
                        timeout 600 ../sim/dartSim/benchmark/DartRollingBenchmark --nogui --dt=$dt --solver=$solver --e=$forcedir
                    done
                done
            else
                echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
            fi
        fi
    done
done
