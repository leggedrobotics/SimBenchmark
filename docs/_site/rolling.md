# Rolling test 

![rolling-test-image](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/RollingBenchmark.png)

Rolling test is for testing frictional contact behavior. The error is measured by comparing the simulation with analytical solution.
The test focuses on:

1. Frictional cone (diagonal, elliptic)
2. The accuracy of frictional contact simulation
3. The violation of hard-contact constraint (penetration)
4. The effect of ERP (error-correcting)

## Test scenario 

- static ground
- 10 kg box on the ground
- 25 number of 1 kg balls on the box
- 150 N force applied to box in the xy direction  

## Result

![rolling-test-error-plot](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-xy.png)

![rolling-test-speed-bar](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/rollingbar.png)


<!---
#### With ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-xy.png)

#### Without ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-xy.png)

## Force along Y-direction 

#### With ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-noerp-y.png)

#### Without ERP

![error-realtimefactor-image-noerp-xy](https://raw.githubusercontent.com/EastskyKang/simbench/master/img/rolling/error-speed-erp-y.png)
--->