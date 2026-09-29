# Trees for hive_base

`pine_tree.dae` and `oak_tree.dae` with textures come from the Gazebo Fuel models
[Pine Tree](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine%20Tree) and
[Oak tree](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak%20tree) by Open Robotics,
licensed CC0 1.0. They are stored here so the world loads offline and the Gazebo GUI does not
have to resolve thousands of online URIs.

The world references them as `/workspace/worlds/trees/...`, i.e. the path where the start
scripts mount the repo inside the container.
