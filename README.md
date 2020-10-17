# Static step validation package

The static step validation package contains two packages used for planning steps on rough terrain. They are based on the rough terrain planner of [IHMC](https://arxiv.org/abs/1907.08673). The *step alidation* library will snap a node to the environment based on its x and y coordinate and yaw orientation. Then it will perform several checks if the desired step is achievable for the robot. This way steps can be validated one by one. The second package, *step expansion*, allows for simultaneous check of large areas. This is a convenient way to check to performance of the step validation library.

## Step validation libray

### Node snapping

The input of this library are footsteps containing the coordinates x, y and yaw. The step will be snapped to the orientation of the highest point under the footprint, seen in the figure below (source [IHMC](https://arxiv.org/abs/1907.08673)). This will give a footstep with coordinates x,y and z as well as the roll pitch yaw orientation.

<img src="/images/node_snapping.png" width="50%">

### Step validation

After snapping the step to the environment several checks are done to ensure the step is valid to execute. These checks can be tuned with parameters found in [footsteps_talos.yaml](./config/footstep_talos.yaml).

#### Check height

The first check is to ensure that the height difference between the stance leg and the swing leg at target position does not exceed the maximum.

#### Check incline
The second check is to ensure the target orientation does not exceed the maximum incline.

#### Step collision

The step collision check ensures the foot will not collide with an obstacle or the ground in the given orientation and position. If a part of the step collides with the environment the step will be invalid.

#### Stance leg clearance

The target position of the swing leg cannot be too close to the position of the stance leg. If this is the case the step is invalid.

#### Shin Collision
To make sure there is no collision with the shin or with the ankle motor at the side of the foot the areas in front and at the side of the target position are checked for obstacles. If there will be collision the step is not valid.

#### Partial foothold

The partial foothold checks if a predifined percentage of the target step is contact with the ground. If this is not the case the step will be invalid.

### Waypoint trajectory calculation

The waypoint trajectory calculation will return a vector of waypoints for a given swing trajectory. It will use the elevation map to avoid hitting obstacles when transitioning from one state to the other.


## Step expansion


This node will check a given area the validity of the steps for every grid point of that area. In [footsteps_talos.yaml](./config/footstep_talos.yaml) this area can be adjusted. The whole map can also be checked. The node will publish a markerlist containing red or green dots on the position of every step indicating whether a step is valid or not. This node is usefull to check the peformance of the step validaton library.
