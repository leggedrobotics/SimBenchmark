#!/usr/bin/env bash

########################################################################################################################
##                                                                                                                    ##
## ANYMAL BENCHMARK                                                                                                   ##
##                                                                                                                    ##
########################################################################################################################

echo "    ___  ________    ___   _____    _______________________"
echo "   /   |/_  __/ /   /   | / ___/   /_  __/ ____/ ___/_  __/"
echo "  / /| | / / / /   / /| | \__ \     / / / __/  \__ \ / /   "
echo " / ___ |/ / / /___/ ___ |___/ /    / / / /___ ___/ // /    "
echo "/_/  |_/_/ /_____/_/  |_/____/    /_/ /_____//____//_/     "


########################################################################################################################
# select sims
########################################################################################################################
source selectsim.sh


########################################################################################################################
# test
########################################################################################################################
csv_file=$( date +"%Y-%m-%d-%H:%M:%S.csv" )

# the number of test (for num_row <= 3)
num_test=10

# the number of anymals
num_row_rai=( 1 2 3 4 5 )
num_row_mjc=( 1 2 3 4 5 )
num_row_ode=( 1 2 3 4 5 )
num_row_bt=( 1 2 3 4 5 )
num_row_dart=( 1 2 3 4 5 )

# feedback (PD control)
feedback=true

# RAI
if [ "$test_rai" == 'ON' ]; then
	if [ "$RAISIM_ON" == "ON" ]; then

		echo "====================================================================="
		echo "RAI"

		for num_row in ${num_row_rai[@]}
		do
			if [[ $num_row -le 3 ]]
			then
				for (( i=1; i <= $num_test; ++i ))
				do
					../sim/raiSim/benchmark/RaiAtlasContactBenchmark --nogui --row=$num_row --csv=$csv_file
				done
			else
				../sim/raiSim/benchmark/RaiAtlasContactBenchmark --nogui --row=$num_row --csv=$csv_file
			fi
		done
	else
		echo "raisim is not built. turn on BENCHMARK_RAISIM option in cmake"
	fi
fi

# BULLET
if [ "$test_bt" == 'ON' ]; then
	if [ "$BTSIM_ON" == "ON" ]; then

		echo "====================================================================="
		echo "BULLET (MULTIBODY)"

		for num_row in ${num_row_bt[@]}
		do
			if [[ $num_row -le 3 ]]
			then
				for (( i=1; i <= $num_test; ++i ))
				do
					../sim/bulletMultibodySim/benchmark/BtMbAtlasContactBenchmark \
					--nogui --row=$num_row --csv=$csv_file
				done
			else
				../sim/bulletMultibodySim/benchmark/BtMbAtlasContactBenchmark \
				--nogui --row=$num_row --csv=$csv_file
			fi
		done
	else
		echo "bulletsim is not built. turn on BENCHMARK_BULLETSIM option in cmake"
	fi
fi

# DART
if [ "$test_dart" == 'ON' ]; then
	if [ "$DARTSIM_ON" == "ON" ] ; then

		echo "====================================================================="
		echo "DART (DANTZIG - BULLET)"

		for num_row in ${num_row_dart[@]}
		do
			if [[ $num_row -le 3 ]]
			then
				for (( i=1; i <= $num_test; ++i ))
				do
					../sim/dartSim/benchmark/DartAtlasContactBenchmark \
					--nogui --row=$num_row --solver=dantzig --detector=bullet --csv=$csv_file
				done
			else
				../sim/dartSim/benchmark/DartAtlasContactBenchmark \
				--nogui --row=$num_row --solver=dantzig --detector=bullet --csv=$csv_file
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
					../sim/dartSim/benchmark/DartAtlasContactBenchmark \
					--nogui --row=$num_row --solver=pgs --detector=bullet --csv=$csv_file
				done
			else
				../sim/dartSim/benchmark/DartAtlasContactBenchmark \
				--nogui --row=$num_row --solver=pgs --detector=bullet --csv=$csv_file
			fi
		done

	else
		echo "dartsim is not built. turn on BENCHMARK_DARTSIM option in cmake"
	fi
fi


# MUJOCO
if [ "$test_mjc" == 'ON' ]; then
	if [ "$MJCSIM_ON" == "ON" ] ; then

		echo "====================================================================="
		echo "MUJOCO (PGS)"
		for num_row in ${num_row_mjc[@]}
		do
			if [[ $num_row -le 3 ]]
			then
				for (( i=1; i <= $num_test; ++i ))
				do
					../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
					--nogui --row=$num_row --solver=pgs --csv=$csv_file
				done
			else
				../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
				--nogui --row=$num_row --solver=pgs --csv=$csv_file
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
					../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
					--nogui --row=$num_row --solver=cg --csv=$csv_file
				done
			else
				../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
				--nogui --row=$num_row --solver=cg --csv=$csv_file
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
					../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
					--nogui --row=$num_row --solver=newton --csv=$csv_file
				done
			else
				../sim/mujocoSim/benchmark/MjcAtlasContactBenchmark \
				--nogui --row=$num_row --solver=newton --csv=$csv_file
			fi
		done

	else
		echo "mujocosim is not built. turn on BENCHMARK_MUJOCOSIM option in cmake"
	fi
fi