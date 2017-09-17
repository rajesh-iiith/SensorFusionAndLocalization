

### Effect of PID Components on Implementation: 

* *P (Proportional) Component*: This is proportional to the cross-track error. A controller with just P coponent is not very stable and it oscillates about the setpoint. 
* *D (Differential) Component*: To get around the issue of oscillation about the set point caused by pure-p controller, a component proportional to the derivative of the cross-track error can be used. 
* *I (Integral) Component*: Despite using both P and D components, the error might converge to a large value. Therefore, we need an additional component simply sums up the cross-track error over time. This results in reduction of proportional gain, which typically cause oscillations while driving at high speeds. It also limits the memory of this term to avoid overshooting.

### Selection of Hyperparameters:

To get an intuitive understanding of the importance of the different components, I decided to tune the parameters manually. 
* Started with a K<sub>p<sub> value, setting K<sub>i<sub> and K<sub>d<sub> = 0
* I kept increasing K<sub>d<sub> until oscillations were reasonably reduced.
* If car going outside permissbile region then alter K<sub>p<sub> and K<sub>i<sub> (increase if cause is slow reactivity, reduce if cause is oscillation).

I finally achieved reasonably good results with K<sub>p<sub> = 0.11, K<sub>i<sub> = 0, K<sub>d<sub> = 2.5. 