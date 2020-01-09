# Controlling a R- <a href="https://www.codecogs.com/eqnedit.php?latex=$\theta$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\theta$" title="$\theta$" /></a> Nonlinear Robot Manipulator
The equation of motions of the system are: <br/>
## Linearizing the model
The problem statement is to control a Theta-R Robot Manipulator. Since the model is non-linear, to control the robot using linear control methods and techniques we linearize the system about a equillibrium point. The equillibrium point for all the simulation is [pi/4  2  0  0]'. 

## Controller Design by Pole Placement
### [State Feedback](StateFeedback_1.m)
After linearization we obtain the state matrices, the eigen values of theses matrices lie in the right half plane indicating that the open loop system is unstable. We use the place command in matlab to place poles of the clodes loop system in the left half plane. The control input is given by (U-Ue) = K*(X-Xe) because the equllibrium point is non zero. The linear controller is then implemented to control the non-linear system and an animation is generated. <br/>
![](demo/StateFeedback_1-gif.gif)
### [Observer Compensator](ObserverControllerCompensator_1.m)
We assume a scenario where we do not have sensors to measure all our states of the system, we implement a luenberger observer to estimate the states. Since our observer is linear, we again have to sybtract Ye and Ue before we feed the output and control input to the observer. We use pole placement to place the poles of (A-LC) in left half plane. As rule of thumb choose the observer poles 2-6 times faster than system poles. Since we havecomplete contrl over the observer we choose any initial condition. For comparison we select it as [0 0 0 0]' for all the codes.
![](demo/ObserverControllerCompensator_1-gif.gif)

## Controller Design using Linear Matrix Inequalities(LMIs)
We use the [CVX](http://cvxr.com/cvx/) solver for solving LMIs, one can also use the Robust Control Toolbox from MATLAB.
### [State Feedback](LMI_StateFeedback_1.m)
We use the lyapunov theory to form a matrix inequality and use congruence transformation to convert this into a Linear Matrix Inequality. This is then appended with a another linear term to ensure faster convergence.
![](demo/LMIStateFeedback_1-gif.gif)
### [Optimal State Feedback](LMI_OptimalStateFeedback_1.m)
We frame the Countinous Algebraic Ricatti Equation as an LMI and solve for feedback gain. <br/>
![](demo/LMIOptimalStateFeedback_1-gif.gif)
### [Observer Compensator](LMI_ObserverControllerCompensator_1.m)
We use the lyapunov theory to form a matrix inequality and use congruence transformation to convert this into a Linear Matrix Inequality. This is then appended with a another linear term to ensure faster convergence. We make the convergence of the observer 4 times faster than the state feedback. We initialise the observer at [0 0 0 0]'
![](demo/LMIObserverControllerCompensator.gif)

## Model Predictive Controller
### [Unconstranied MPC]()
![](demo/MPCUnconstrained_1-gif.gif)
<!--- #### [Unconstrained MPC with Observer]()
![](demo/MPCUnconstrainedObserver_1-gif.gif) --->



## Ackermann Steering<br/>
#### [Ackermann Simulator](Ackermann_Simulator.m)
The code simulates a very simplified kinematic model of ackermann steering commonly used in automobiles.<br />
Ref link: https://www.xarg.org/book/kinematics/ackerman-steering/<br /><br />
A simple PD controller is used to main a constant lateral distance. For the sake of simplicity we keep the forward velocity constant during the control action. The objective of this code was to biuld an intuition for tuning PD conitrol of an ackermann steered vehicle.<br />
Controller:<br /> Its worthwhile to mention that steering angle is not the same as car angle. Our control input is the steering angle not car angle, The objective of controller(100Hz) is slow the steering angle enough so that car angle catches up to the steering angle. To achieve this we have the Derevative component of PID. A dead band is also added in the middle, though it is not neccesary. Car dimensions to a 1/10 scale of an actual car.<br /><br />
Markers:<br /> Box -- Car<br /> Light Green line -- Car Angle (theta)<br /> Blue Line -- Steering Angle (phi)<br />Red Line -- Lateral step or goal<br /> All dimensions are in meters. Car angle and Steering angle plotted at origin.<br /><br /> 
Output:<br /> To record a video change "vid_recorder" from 0 to 1. The file name by default is 'ackermann_simulator.avi'.<br /><br />
Testing:<br />        1) Kp = 1.5, Kd = 25  demos a working control ( but not best)<br /> 2) Kp = 1.5, Kd = 30  faster settling time but with steady state error<br /> 3) Kp = 1.5, Kd = 5   demos the need of Kd<br /> 4) change 'speed' to visualize the affect of car speed on controller<br /> 5) change 'goal' to change the goal or lateral step of the car<br /><br />
Remarks:<br /> The setting point as default is 1.5 m given that car width is  only 0.15m the goal is extermenly large for any controller to reach. A more practical value of 'goal' would be 0.5m.<br /> 
![](demo/AckermannSimulation-gif.gif)
##### For latex rendering: https://www.codecogs.com/latex/eqneditor.php

