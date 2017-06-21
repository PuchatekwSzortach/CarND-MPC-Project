### MPC Projects

Following notes detail how I arrived at scaling factors of different cost function parameters once my basic MPC implementation was in place.

Initially I started with cost function made of only x, y and psi components, that is dropping components that try to minimize use of actuators and jerky movements. My reference velocity was 40 and actuation delay to 0. Also initial state was set to reported car state, that is no extrapolation into future was made to recompensate for actuation delay - as we didn't have any.

#### All cost term scales set to 1, cte, epsi, v terms only
This worked reasonably well on straight road, though a lot of small acceleration and brakes ('pumping the brake pedal') were noticeable. Also on large curves car wobbled between both sides of correct path, eventually falling of the track on one of last sharp turns.

Since car had too large error in both cte and epsi on sharp turns, I needed to bump up their numbers. With velocity set to 40, cost due to velocity could easily be 1600, while for cte and epsilon numbers are much closer to 10~20 range. Therefore I scaled both cte and epsi by 100.

#### cte scale = 100, epsi scale = 100
Above parameters resulted in much smoother ride. Some minute wobbling was still there, but hardly noticeable. However as expected car still kept on rapidly changing between accelerometer and brake around reference velocity.

In next step I added cost terms that minimize use of actuators and try to enforce smooth actuations. All terms were kept with default scale of 1.

#### cte scale = 100, epsi scale = 100, actuation cost terms included
Even using default scale for actuation cost terms smoothes out the ride. Vehicle goes around the track at speed just slightly under reference velocity, only lightly tapping on brake through steep curves.

This solved the problem for control with no delay.
Next step was to re-enable the delay.

#### cte scale = 100, epsi scale = 100, actuation cost terms included, delay included
Adding actuation delay immediately wrecks havoc with the model. It immediately starts wobbling around the track and falls off it after a few seconds.

To help us tackle delay problem, we can try to predict car state after actuation delay and set that to initial state for MPC solver. Since we don't know steering and acceleration values, we will ignore them. Also as we aligned x-axis to where car faces, we only need to compute change in car x position due to longitudinal velocity. Then we also calculate cte and epsi at new x position.

Updating model that way indeed helps it drive better, but it still wobbles and eventually falls off the track, though after a significantly longer distance than before.

Therefore next step was to scale up cte and epsi cost gains again. I chose to simply increase them 10-folds.

#### cte scale = 1000, epsi scale = 1000, actuation cost terms included, delay included
This didn't help at all. In fact previous gains seemed to give somewhat better results.
Since model was swining too wildely, I decided to bump up cost terms due to steering changes.

I kept on bumping steering terms costs up to 100. At that point model behaved better and was able to drive till first turn. Increasing steering terms further didn't help.
