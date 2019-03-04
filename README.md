# The Crawl Planner

This is a very basic static walking planner

## To run:
start VMplan_controller with trunkcontroller

to accept the new plan
```
$ executePlan
```
Launch the planner pipeline:
```
$ roslaunch dls_planner start.launch
```

change planner:
```
$ changePlanner crawl_planner
```

per vedere i messaggi rostopic echo /hyq/planner_debug e in lcm with ROBOT_PLANNER_DEBUG
Start the simulator, and switch to DWLPlanController or VMOptimizationController

the plan is saved in a local plan plannes_ws e poi viene salvato
nella traiettoria planned_wt che viene pubblicata in blocco
actual_state è disponibile nello starting, la init è come la construct del supervisor



