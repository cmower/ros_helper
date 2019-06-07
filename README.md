# ros_helper

A collection of helper scripts and nodes for ros. 

## Todo

* Sort out `__all__` in `src`
* Update `rviz_spray_sim` to use `ros_helper`
* Update naming conventions in scripts, e.g. do we need `_node` at end?
* Consider converting all use of lists to np arrays, see [here](https://www.freecodecamp.org/news/if-you-have-slow-loops-in-python-you-can-fix-it-until-you-cant-3a39e03b6f35/). This may have an adverse affect however, I am not sure if ROS will handle np arrays when serializing msgs. Will need to test. 
* Update `keyboard_node` to use `simple_pub_sub`
