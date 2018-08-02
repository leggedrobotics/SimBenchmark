# Libraries

add the graphics environmental variable as
echo 'export RAI_GRAPHICS_OPENGL_ROOT='$PWD/raiGraphics'' >> ~/.bashrc

This directory is for external libraries: 

- yaml-cpp 0.6.2
	- for loading test parameters from yaml files. 
- RaiCommons 
	- common features including plot, log etc.
- RaiGraphics
	- graphic user interface for benchmark test instances
- ~~RaiSim~~ 
- Bullet Physics 2.88
	- website: http://bulletphysics.org/
    - repository: https://github.com/bulletphysics/bullet3
- ODE 0.15.2
	- website: http://www.ode.org/
    - wiki: https://www.ode-wiki.org/
- MUJOCO 1.50
	- website: http://mujoco.org/
- Dart 
	- website: https://dartsim.github.io/
	- repository: https://github.com/dartsim/dart

## Note  

- MuJoCo is proprietary software. Purchase a license and put mjkey.txt into mjproXXX directory to benchmark MuJoCo.
- RaiSim is currently unreleased and only the researchers at RSL, ETH Zurich have an access. 
