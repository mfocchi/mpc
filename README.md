# The Simple Planner

This is a very basic static walking planner that relies on hard-coded primers to move the CoM and foot position of the robot. 
It is slow, not optimized, and lacks fine direction or speed control, but it should be a good example.

## To run:

Start the simulator, and switch to DWLPlanController or VMOptimizationController
```
$ changeController 
```

Next, start the planner:
```
roslaunch dls_simple_planner simple_planner.launch
```

## To control:
In order to control the generate simple planner use the dls_console:

```
rosrun dls_console dls_console
```

rosrun dls_console dls_console
Menu options:
(to send command use - rosrun dls_console dls_console) 
(topic should be /robot/char) 
- f to move forward 
- b to move backwards 
- c to move left 
- v to move right 
- 0 to stop
- d to debug (then take single state machine steps with s)
- s to step state machine once
- i to print information
```