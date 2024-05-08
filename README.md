# ho_planning


![](RRT*.png)

To run the project 
## 1. 
```sh
roslaunch ho_planning stonefish.launch # launch file name might change
```

## 2.
```sh
rosrun ho_planning dwa_node.py # testing phase
```

## 3.
```sh
rosrun ho_planning global_path_planner_node.py #testing phase
```

## 4.
```sh
# In the turtlebot_integration.launch file, change
    <include file="$(find turtlebot_simulation)/launch/kobuki_basic.launch">
    #to
        <include file="$(find turtlebot_simulation)/launch/turtlebot_basic.launch">
        # for the intervention tasks
#We will put all the commands in one launch file once all the implementations are done
```

