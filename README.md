# ros_helper

The `ros_helper` package provides various helper classes/methods/nodes/messages
for [ROS1](https://www.ros.org/). Over the years, I find myself repeating a lot
of code when developing ROS packages. As a result, I started to collect these
common snippets into one helpful package.

Feel free to clone and use `ros_helper` inside your catkin workspace, or
copy-paste/modify any of the code/scripts for your projects.

Install
1. Clone `ros_helper` into catkin workspace.
1. `cd ros_helper`
1. Install deps: `$ rosdep update ; rosdep install --from-paths ./ -iy`
1. Build: `$ catkin build -s`

Run tests using
```
$ catkin build ros_helper -s --catkin-make-args run_tests
```
