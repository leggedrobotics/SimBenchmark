# SimBenchmark

The project is for benchmarking the state-of-the-art rigid body simulation engines for contact dynamics solving:

- RaiSim (Jemin Hwangbo, Dongho Kang et al.)
- [Bullet Physics](http://bulletphysics.org/)
- [Open Dynamics Engine](http://www.ode.org/)
- [Multi-Joint dynamics with Contact (a.k.a. MuJoCo)](http://mujoco.org/)
- [DART Sim](https://dartsim.github.io/)

You can see the details and the result in [webpage](https://leggedrobotics.github.io/SimBenchmark/).


## Overview

- The project is consist of common part(interface) and libraries which corresponds each simulation engine.

## Installation

- Matlab R2018a is required for plotting
- MuJoCo is proprietary. You need a license for testing MuJoCo.
- RaiSim is proprietary, and currently unreleased.

### Install with bash script

- Run install.sh script by ```./install.sh ```
- The dependencies are downloaded in ```lib``` directory.

### Install manually

- install Bullet Physics
    - turn on double precision and shared library option
    - Build and install library into local
- install ODE
    - Build and install library into local
- install raiGraphics
    - Build install library into local
- install raiCommons
    - Build install library into local
- download MuJoCo v.1.5 in lib directory
    - ```lib/mjpro150```
    - put ```mjkey.txt``` in ```lib/mjpro150/mjkey.txt```

## Test scenario

Go to benchmark directory for details

## Contact

Please feel free to contact Dongho Kang(kangd@ethz.ch) if you have any concerns or suggestions.
