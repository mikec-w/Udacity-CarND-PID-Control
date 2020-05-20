# Udacity Self-Driving Car Nanodegree
## PID Control Project

### Introduction

This project requires the implementation of a PID steering angle controller to drive the simulated vehicle around the lakeside test track. It's control input is the Cross Track Error (CTE) and the car must not leave the track. 

The initial development was performed with a fixed throttle position of 30% although this was substituted with a PID controller targeting 35mph.

### The PID Controller

The PID controller was implemented as a seperate class allowing it be to reused for both steer and throttle. An initialisation function was provided that allowed the appropriate proportional, integral and derivative gains to be set. 

This object was subsequently passed the CTE for each time step and internally calculated the single-step derivative and integral of this value before outputing a steer angle for the simulator using the following:

`return Kp * p_error + Ki * i_error + Kd * d_error;`

In order to assist in the tuning of the controller, the object was modified to also calculate the RMSE (root mean squared error) of the CTE value. A lower number indicating better tracking. 

As this PID class was also used for the throttle control, a Feed Forward term was included with a small modification to the output:

`return Kp * p_error + Ki * i_error + Kd * d_error + FF;`

As the throttle controller needs to overcome drag at a given speed, the use of the feed forward (for example, 30% throttle at 35kph) means that the P, I and D gains can be reduced significantly and focus on controlling around the target. For the steer controller, the feed forward is set to zero. 

Finally the PID output is clipped to the limits of the steering angle (+/- 1) and the throttle (0 to 1)
### Initial tuning

For the initialling tuning a manual process was adopted starting with just a P gain. Here the process was an attempt to have as high a P gain as possible to make the car responsive to the track but lower enough to ensure stability. An unstable car being easy to spot as any osciallations around target will grow and quickly result in the car going off track. 

From the first few runs it is clear that a simple P controller is not sufficient for this use case. A high enough P to be able to generate large enough steering for tight corners was not stable. A low enough P to be stable would understeer off the track. There are solutions to this mentioned below.

The next step was the Integral gain. Here the intend is to remove any long term systemic error in the car position. Too high an integral would result in the car slowly wandering back and forth around the centre while too low an integral could result in the car sitting slightly off centre. 

In this particular example a very small integral gain was required to gentle nudge the car back to centre. 

Finally the derivative gain was tuned. Here the derivative is necessary to drive the vehicle around the tighter corners by applying more steering angle as the error changes rapidly. Again here is a balance between stability and responsiveness. In this case the P and the D gains are actually closely coupled due to the nature of the problem. The D gain was increased to the point where the car could navigate even the tightest corners without over correcting. 

Using the manual method, the following gains were achieved. These were capable of driving the car around the track without hitting either lane marking.

P = 0.2
I = 0.002
D = 2.0

### Twiddle

Having performed the manual tuning, the twiddle algorithm was implemented for fine tuning. This uses the RMSE value as a target to minimise and further honed on the values by repeated simulation.

The algorithm effectively twiddles the gains up and down until it finds the best error value. 

In this example it is quite a time consuming process as each of the three gains need to be tuned and it is important to drive enough of the track to make the RMSE value valid. 

Each time the algorithm updates the gains, the RMSE and integrators are reset.

Using Twiddle, the revised gains only slightly changed the values to give:

P = 0.216
I = 0.002
D = 2.145


### Throttle PID

The throttle controller was a simpler affair once the Feed Foward term had been introduced. This effectively gives a static offset to the output meaning the other gains do not have to work as hard to control speed. As the vehicle speed is a little more stable, the P gain is all that is really required with a small integral just to correct for any slight error in the feed forward value.


### Controller Improvements

While the manual tuning and twiddle algorithm gave an acceptable performance, there are certain limitations that need to be noted. Firstly, the tuning was performed using a speed controller set to 35kph. Changing this by even a small amount will alter these gains. 

Furthermore, as already mentioned, a high proportional gain that could be used to steer around the tighter corners resulted in an unstable controller. Here it would be recommended to have a variable P gain based on CTE. When CTE is low, the P gain would also be low reducing the slight oscillation seen around the centre lane when the car tries to correct position. When the CTE increases, such as on corner entry, the authority of the P gain becomes larger. 

A similar approach could also be employed with the I and D gains. 

With the Integrator, at present the integrator is always running. This is not ideal and a particular example of the issue with it can be seen as the car initially starts to accelerate. As the car is moving slowly, the controller is slow to return the car to the centre line - here the integrator can start to wind up even though it is not a systemic error. To avoid this, the integrator should only be enabled/updated when the car speed is close to its target.

Finally, it is clear that there is a speed dependency on the gains and as such the P, I and D should be modified dependent on the car speed. 

While these modifications would enable a more robust controller to be developed, it does have the downside of significantly complicating the Twiddle algorithm. 


### Results

The implemented algorithm performed as required at 35mph. Higher speeds were possible with this controller architecture up to around 75mph with adjusted gains but it is unlikely an occupant would have found it that comfortable. A revised architecture would reach 100mph with a much smoother trajectory. 

