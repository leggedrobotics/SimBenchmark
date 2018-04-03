# MUJOCO_SIM

MUJOCO_SIM is our implementation of physics simulator based on MuJoCo engine. 

Note that 

- run time model generation is not possible. 
- MuJoCoSim only can load the simulation model from .xml or .urdf script.
- Setting states of simulation object is also not possible. 
    - However external force/torque can be applied to each body and joints.  
- Only getting states of object is possible.