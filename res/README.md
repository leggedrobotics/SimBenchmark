# Resources

This directory contains resource files (incl. URDF, mujoco XML script etc.) for simulation. 

+ mjkey.txt file (put your mjkey.txt file in ./mujoco directory)

## Note 

- MuJoCo uses URDF or MJCF(XML) file format to load a simulation world. All object is defined in one file. 
- Bullet uses URDF file format to load objects. Each object corresponds to one URDF file. 
- ODE, RAI, DART uses URDF to load articulated robot system.  

## ANYmal 

ANYmal is 18-DOC quadrupedal robot developed by robot researchers of [ETH RSL](http://www.rsl.ethz.ch/robots-media/anymal.html).
The URDF file of ANYmal is provided from ETH RSL. 

## The list of files

- ANYmal-PD-benchmark

- ANYmal-energy-benchmark
 	- URDF for AnymalEnergyBenchmark
 	- revolute joint -> continous joint (no joint limit) to prevent energy loss
 	- no collision bodies to prevent internal collision (or set "disable contact")
- 2DRobotArm: fixed based robot arm 
- ANYmal: 18-DOF quadrupedal robot from [ETH RSL](http://www.rsl.ethz.ch/robots-media/anymal.html)
- ANYmal-nomesh: ANYmal URDF no mesh version
- ANYmal-energy: ANYmal URDF with no joint limit 
- Atlas: bipedal robot Atlas from Boston Dynamics  
- Multibody: 9-DOF parts(base and one leg) of ANYmal for testing multibody simulation
- Singlebody: Single body objects for basic tests