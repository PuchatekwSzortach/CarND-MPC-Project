### MPC Projects

Following notes detail how I arrived at scaling factors of different cost function parameters once my basic MPC implementation was in place and how I solved actuation delay problem.

Initially I started with cost function made of only cte, epsi and velocity components, that is dropping components that try to minimize use of actuators and jerky movements. My reference velocity was 40 and actuation delay 0. Also initial state was set to reported car state, that is no extrapolation into future was made to recompensate for actuation delay - as we didn't have any. Vehicle model presented in Udacity class was used in the solver.

Solves predicts 1sec into future with time steps of 0.1sec. At high speeds data path provided by waypoints around curve would easily be covered in 1sec, thus predicting further ahead wouldn't make much sense. Also we luckily have no on-road obstacles to worry about.

#### All cost term scales set to 1, cte, epsi, v terms only
This worked reasonably well on straight road, though a lot of short bursts of acceleration and braking were noticeable. Also on large curves car wobbled between both sides of correct path, eventually falling off the track on one of last sharp turns.

Since car had too large error in both cte and epsi on sharp turns, I needed to bump up their numbers. With velocity set to 40, cost due to velocity could easily be 1600, while for cte and epsilon numbers are much closer to 10~20 range. Therefore I scaled both cte and epsi by 100.

#### cte scale = 100, epsi scale = 100
Above parameters resulted in much smoother ride. Some minute wobbling was still there, but hardly noticeable. However as expected car still kept on rapidly changing between accelerating and breaking around reference velocity.

In next step I added cost terms that minimize use of actuators and try to enforce smooth actuations. All terms were kept with default scale of 1.

#### cte scale = 100, epsi scale = 100, actuation cost terms included
Even using default scale for actuation cost terms smoothes out the ride. Vehicle goes around the track at speed just slightly under reference velocity, only lightly tapping on brake through steep curves.

This solved the problem for control with no delay.
Next step was to re-enable the delay.

#### cte scale = 100, epsi scale = 100, actuation cost terms included, delay included
Adding actuation delay immediately wrecks havoc with the model that doesn't take it into account. Vehicle quickly starts wobbling around the track and falls off it after a few seconds.

To help us tackle delay problem, we predict car state after actuation delay and set that to initial state for MPC solver. New state can be computed based on vehicle model equations. 
I made a slight change to vehicle update equations here. Vehicle model we were taught in the class didn't consider effects of longitudinal and radial acceleration on car position after time delay. I included effects of longitudinal and radial acceleration when predicting intial vehicle position after an actuation delay. It would be even better to use that model for solver too, but I didn't do so.

Resulting model was better than original model that didn't account for delay, but it still drove off the track after a few dozen metres. Its steering actuations very widely jumping between extreme left and right steering. To solve that I significantly increased angle-related cost terms, that is epsi cost, cost minimizing steering and cost smoothing steering. I made some minor changes to acceleration gains as well.

#### Final parameters: cte scale = 100, epsi scale = 2000, steering minimization scale = 10000, acceleration minimization scale = 10, steering smoothness scale = 10000, acceleration smoothness scale = 1
With above cost gains model drives smoothly around the whole track.
