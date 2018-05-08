# ANYmal PD control test

![anymal-test-image](../img/AnymalBenchmark.png)

ANYmal test is for testing articulated robot system. In this test, the 18 DOF quadruped robot [ANYmal](http://www.rsl.ethz.ch/) is feedback-controlled to stand on the flat ground.  

The test focuses on:

1. Speed of the articulated system (multibody system with revolute joints) simulation
2. Scalability of the simulators

## Test scenario 

## Tested solver list

- Rai 
    - Bisection solver (Rai solver)
- Bullet
    - Multibody solver (Featherstone implementation on Sequential impulse)
- ODE
    - Standard
- MuJoCo
    - PGS
    - CG
    - Newton 
- Dart
    - Dantzig
    - PGS

Note that ODE quick solver easily fails for articulated system simulation as the following video, thus was exempted. 
 
<div style="position:relative;padding-top:75%;">
    <iframe src="https://www.youtube.com/embed/X0lYN7bzoNk?rel=0&amp;controls=0&amp;showinfo=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen
 style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>

According to the manual this is due to the solver's property:
 
> QuickStep is great for stacks of objects especially when the auto-disable feature is used as well. However, it has poor accuracy for near-singular systems. Near-singular systems can occur when using high-friction contacts, motors, or certain articulated structures. For example, a robot with multiple legs sitting on the ground may be near-singular. 
>
> *from [ODE wiki](https://www.ode-wiki.org/wiki/index.php?title=Manual:_World)*

## Results 

The results of the test are as following figures.

![anymal-test-speed-bar](../img/samplebar.png)

![anymal-test-speed-plot](../img/sampleplot.png)

![anymal-test-speed-plot-log](../img/sampleplot-log.png)
