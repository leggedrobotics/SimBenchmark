# Simbench 

## Overview 

Simbench is the project for benchmarking rigid body simulation with contacts.

The list of the simulators benchmarked is as follows:

- RaiSim by Jemin Hwangbo, Dongho Kang et al.
- [Bullet Physics](http://bulletphysics.org/)  by Erwin Coumans
- [Open Dynamics Engine](http://www.ode.org/) by Russell Smith
- [Multi-Joint dynamics with Contact (a.k.a. MuJoCo)](http://mujoco.org/) by Emanuel Todorov
- [DART Sim](https://dartsim.github.io/)

## Test list

- Rolling test
- Bouncing test
- Tower test
- ANYmal PD control test

# Benchmark Test Results

## Rolling test 

![rolling-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/RollingBenchmark.png)

Rolling test is for testing frictional contact behavior. The error is measured by comparing the simulation with analytical solution.
The test focuses on:

1. Frictional cone (diagonal, elliptic)
2. The accuracy of frictional contact simulation
3. The violation of hard-contact constraint (penetration)
4. The effect of ERP (error-correcting)

### Force along XY-direction 

#### With ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-xy.png)

#### Without ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-erp-xy.png)


### Force along Y-direction 

#### With ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-y.png)

#### Without ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-erp-y.png)



## ANYmal test

![anymal-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/AnymalBenchmark.png)

ANYmal test is for testing articulated robot system. In this test, the 18 DOF quadruped robot [ANYmal](http://www.rsl.ethz.ch/) is feedback-controlled to stand on the flat ground.  

The test focuses on:

1. Speed of the articulated system (multibody system with revolute joints) simulation
2. Scalability of the simulators

The test result is as the following plots.

### Speed of 1 robot simulation 
![anymal-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/anymal-stand/samplebar.png)

### Time for 50k iteration 
![anymal-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/anymal-stand/sampleplot.png)

![anymal-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/anymal-stand/sampleplot-log.png)
