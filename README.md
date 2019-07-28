# SimBenchmark

SimBenchmark is a benchmark suite for state-of-the-art physics engines.

We evaluated a few widely used physics engines for robotics and machine learning applications. 
The list of the engines is as follows:

- [RAISIM](https://github.com/leggedrobotics/raisimLib)
- [Bullet Physics](http://bulletphysics.org/)
- [Open Dynamics Engine](http://www.ode.org/)
- [Multi-Joint dynamics with Contact (a.k.a. MuJoCo)](http://mujoco.org/)
- [DART Sim](https://dartsim.github.io/)

You can see the details in [webpage](https://leggedrobotics.github.io/SimBenchmark/).

![ANYmal PD Control Benchmark](https://leggedrobotics.github.io/SimBenchmark/about/anymal.gif)

## Installation

SimBenchmark runs on Ubuntu 16.04 LTS.  

Notes:
- RAISIM is released as a free-of-charge closed-source library.
- MuJoCo is under a proprietary license. 
- Matlab R2018a is required for plotting (optional). 

### via Docker (recommended)

See [docker/README.md](https://github.com/leggedrobotics/SimBenchmark/blob/master/docker/README.md)

### via bash script 

- Run install.sh script by ```./install.sh ```
- The dependencies are downloaded in ```lib``` directory.
- To use visualization, add environmental variable "RAI_GRAPHICS_OPENGL_ROOT" that points to lib/raiGraphics
- download MuJoCo v.1.5 in lib directory
    - ```lib/mjpro150```
    - put ```mjkey.txt``` in ```lib/mjpro150/mjkey.txt```

## Test and Results

We designed the following tests for the evaluation 
- Rolling test: friction model test
- Bouncing test: single-body elastic collision test
- 666 balls test: single-body hard contact test
- Elastic 666 balls test: single-body energy test
- ANYmal PD control test: articulated-robot-system speed test for quadrupedal robot
- Atlas PD control test: articulated-robot-system speed test for bipedal robot
- ANYmal momentum test: articulated-robot-system momentum test
- ANYmal energy test: articulated-robot-system energy test

Please see [our webpage](https://leggedrobotics.github.io/SimBenchmark/) for more details. 

<!-- ## Citation

If you want to refer the benchmark result in an academic publication, please consider citing as 
 -->
## Contact

- Note that this benchmark is done by Dongho Kang and Jemin Hwangho who are the developers of RaiSim.
- If you have any concern, please contact [Dongho Kang](mailto:kangd@ethz.ch)
