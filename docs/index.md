# Simbench 

## Overview 

Simbench is the project for benchmark of rigid body simulation with contacts.

The list of the simulators is as follows:

- RaiSim by Jemin Hwangbo, Dongho Kang et al.
    - What is RaiSim? RaiSim is a rigid body simulator developed by researchers at RSL, ETH Zurich (Jemin Hwangbo and Dongho Kang). RaiSim features efficient recursive algorithms for articulated systems as well as the new novel contact solver described in [1]. The goal of RaiSim project is to provide simulation for data-driven robotics and animation research. Since the robotics community needs quality data that reflects the reality, RaiSim is designed for uncompromising accuracy. It is also designed to be nearly tuning-free to ease the pain of manual parameter tuning. All the systems presented here are simulated with the default setup. RaiSim is currently unreleased and only the researchers at RSL, ETH Zurich have an access.
- [Bullet Physics](http://bulletphysics.org/) v2.87 by Erwin Coumans
- [Open Dynamics Engine](http://www.ode.org/) by Russell Smith
- [Multi-Joint dynamics with Contact (a.k.a. MuJoCo)](http://mujoco.org/) by Emanuel Todorov
- [DART Sim](https://dartsim.github.io/)

## Test results

- [Rolling test](rolling/rolling.html) - friction model test
- [Bouncing test](bouncing/bouncing.html) - elastic/inelastic collision test
- [666 balls test](666/666.html) - hard contact constraint test
- Building test
- [ANYmal PD control test](anymal/anymal.html) - articulated system speed test
- ANYmal zero gravity test - articulated system momentum test 
- ANYmal free drop test - articulated system momentum test

## Commentary 

If you want to contribute the project or have any concerns, please contact to [Dongho Kang](mailto:kangd@ethz.ch).
