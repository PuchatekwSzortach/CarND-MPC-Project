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
