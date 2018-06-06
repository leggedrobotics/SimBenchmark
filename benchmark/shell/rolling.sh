#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ROLLING TEST                                                                                                       ##
##                                                                                                                    ##
########################################################################################################################

echo "    ____  ____  __    __    _____   ________   _______________________"
echo "   / __ \/ __ \/ /   / /   /  _/ | / / ____/  /_  __/ ____/ ___/_  __/"
echo "  / /_/ / / / / /   / /    / //  |/ / / __     / / / __/  \__ \ / /   "
echo " / _, _/ /_/ / /___/ /____/ // /|  / /_/ /    / / / /___ ___/ // /    "
echo "/_/ |_|\____/_____/_____/___/_/ |_/\____/    /_/ /_____//____//_/     "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
dt_array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" )
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

echo ""
echo "====================================================================="
for dt in ${dt_array[@]}
do
    for forcedir in xy y
    do
        # rai sim
        if [ "$test_rai" == 'ON' ]; then
            if [ "$RAISIM_ON" == "ON" ]; then
                for erpon in true false
                do
                    timeout 600 ../sim/raiSim/benchmark/RaiRollingBenchmark \
                    --nogui --erp-on=$erpon --dt=$dt --force=$forcedir --log --csv=$csv_file
                done
            else
                echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
            fi
        fi

        # bullet sim
        if [ "$test_bt" == 'ON' ]; then
            if [ "$BTSIM_ON" == "ON" ]; then
				for erpon in true false
				do
					timeout 600 ../sim/bulletMultibodySim/benchmark/BtMbRollingBenchmark \
					--nogui --erp-on=$erpon --dt=$dt --force=$forcedir --log --csv=$csv_file
				done
            else
                echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
            fi
        fi

        # ode sim
        if [ "$test_ode" == 'ON' ]; then
            if [ "$ODESIM_ON" == "ON" ] ; then
                for solver in std quick
                do
                    for erpon in true false
                    do
                        timeout 600 ../sim/odeSim/benchmark/OdeRollingBenchmark \
                        --nogui --erp-on=$erpon --dt=$dt --solver=$solver --force=$forcedir --log --csv=$csv_file
                    done
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
                        timeout 600 ../sim/mujocoSim/benchmark/MjcRollingBenchmark \
                        --nogui --dt=$dt --solver=$solver --force=$forcedir --log --csv=$csv_file --integrator=$integrator
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
                    timeout 600 ../sim/dartSim/benchmark/DartRollingBenchmark \
                    --nogui --dt=$dt --solver=$solver --force=$forcedir --log --csv=$csv_file
                done
            else
                echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
            fi
        fi
    done
done

echo "Rolling test finished."