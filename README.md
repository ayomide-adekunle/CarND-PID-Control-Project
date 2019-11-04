# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction
PID Contoller is Proportional-integeral-Derivate controller. These three controllers are used generate control signals just like a vehicle uses steering, throttle, and brake to generate a safe path.

The goal of this project is to implement a PID Controller in C++ to manuver a vehicle around the given track. the Cross track error and velocity is provided by the simulator.

# PID Controller components

## Cross Track Error
A cross track error is the difference between the position of a car from its intended trajectory. For a car to get to its intented trajectory, it has to be steered in proportion to the cross track error.

## P Component
This component gives the steering angle in proportion to the CTE with a proportioanl factor, tau.
```
P = -tau * cte
```
The P component has the direct effect on the car's behavior. It makes the car to steer proportionally to the car's distance from the center of the lane

## D Component
D is a differential component of the controller that gives the temporal derivative of error. 

The D component counteracts the P component's tendency to ring and overshoot. Overshooting will be solved if a D component is properly turned.

```
diff_cte = cte - prev_cte
prev_cte = cte
D = - tau_d * diff_cte
```

## I Component
I-Component is the integral or sum of error to deal with systematic biases. I-Compoenet counteracts a bias in the CTE which prevents the P-D controller from reaching the center line.

```
intgr_cte += cte
tau_i * intgr_cte
```

Combining the three conrollers we have:

```
cte = robot.y
diff_cte = cte - prev_cte
prev_cte = cte
int_cte += cte
steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
```
## Effect of P,I, and D controllers in my Implementation
The P component has the direct effect on the car's behavior. It makes the car to steer proportionally to the car's distance from the center of the lane.I got the P value with oscillating behaviour when I set the value to 0.15.

The D component counteracts the P component's tendency to ring and overshoot. Overshooting will be solved if a D component is properly turned.  I set the D value to 1.29 in my implementation, this value helped the car not to overshoot.

I-Component is the integral or sum of error to deal with systematic biases. I-Compoenet counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. I set this value to 0.00031 is my implementation.

## Choosing the hyperparameters
I take an initial guess for the Kp, Ki, and Kd values and observed the effect of the car. I first make sure I tuned the Kp value appropiately because it has more effect on the behavior of the car. then I tuned Ki and Kd respectively, I did all the tuning manually (trial and error). the values i frist used for my parameters are:
```
{0.05, 0.0001, 1.6}
```
The car drives well bu I see it couldn't take sharpt turns properly, so I increased the P value significanlty,reduced the D value and end up with:
```
pid.Init(0.15, 0.00031, 1.29);
```
