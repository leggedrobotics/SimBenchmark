#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## BOUNCING BENCHMARK                                                                                                 ##
##                                                                                                                    ##
########################################################################################################################

echo "    ____  ____  __  ___   _____________   ________   _______________________"
echo "   / __ )/ __ \/ / / / | / / ____/  _/ | / / ____/  /_  __/ ____/ ___/_  __/"
echo "  / __  / / / / / / /  |/ / /    / //  |/ / / __     / / / __/  \__ \ / /   "
echo " / /_/ / /_/ / /_/ / /|  / /____/ // /|  / /_/ /    / / / /___ ___/ // /    "
echo "/_____/\____/\____/_/ |_/\____/___/_/ |_/\____/    /_/ /_____//____//_/     "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
dt_array=( "0.00001" "0.00004" "0.0001" "0.0004" "0.001" "0.004" "0.01" "0.04" "0.1" )
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

echo ""
echo "====================================================================="
# benchmark test
for dt in ${dt_array[@]}
do
    for e in "1.0"
    do
        # rai sim
        if [ "$test_rai" == 'ON' ]; then
            if [ "$RAISIM_ON" == "ON" ]; then
                for erpon in true false
                do
                    ../sim/raiSim/benchmark/RaiBouncingBenchmark \
                    --nogui --erp-on="$erpon" --dt=$dt --e="$e" --csv=$csv_file
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
					../sim/bulletMultibodySim/benchmark/BtMbBouncingBenchmark \
					 --nogui --erp-on="$erpon" --dt=$dt --e="$e" --csv=$csv_file
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
                        ../sim/odeSim/benchmark/OdeBouncingBenchmark \
                        --nogui --erp-on="$erpon" --dt=$dt --solver=$solver --e="$e" --csv=$csv_file
                    done
                done
            else
                echo "odesim is not built. turn on BENCHMARK_ODESIM option in cmake"
            fi
        fi

        # mujoco sim
        if [ "$test_mjc" == 'ON' ]; then
            echo "bouncing test is not available for mujoco."
        fi

        # dart sim
        if [ "$test_dart" == 'ON' ]; then
            if [ "$DARTSIM_ON" == "ON" ] ; then
                for solver in dantzig pgs
                do
                    # note dart has no erp
                    ../sim/dartSim/benchmark/DartBouncingBenchmark \
                    --nogui --dt=$dt --solver=$solver --e="$e" --csv=$csv_file
                done
            else
                echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
            fi
        fi
    done
done

echo "Bouncing test is finished."