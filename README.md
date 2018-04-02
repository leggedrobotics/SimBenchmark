# sim-benchmark

Benchmark the following state-of-the-art rigid body simulators for contact dynamics solving: 

- raiSim (Jemin Hwangbo et al.)
- Bullet Physics (Erwin Coumans)
- Open Dynamics Engine 
- Multi-Joint dynamics with Contact

**Matlab R2017b is required for plotting**

## Overview 

- The project is consist of common part(interface) and libraries which corresponds each simulation engine. 

## Installation

### Install with sh script

- run install.sh script 

### Install manually 
 
- install Bullet Physics
    - turn on double precision and shared library option 
    - install library into local by ```sudo make install```
- install ODE
    - install library into local 
- install raiGraphics 
    - install library into local 
- install raiCommons 
    - install library into local
- mujoco150 is included in project
    - mjkey.txt is only valid for Dongho Kang
    
## Test scenario

Go to benchmark directory for details