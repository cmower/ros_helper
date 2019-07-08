# ros_helper

A collection of helper code for [ROS](https://www.ros.org/). For instructions, explanations, and examples see the wiki [here](https://github.com/cmower/ros_helper/wiki) (unfinished!). The main reason this package exists is that writing packages/modules for ROS tends to lead to a lot of code repetition; this package attempts to address this issue. `ros_helper` also provides a number of other facilities that are "helpful"!

# Examples

## Publish spherical markers

Goal is to write a Python script that  publishes a sphere marker to Rviz, as shown in the image. This assumes you want to publish a red marker at 20hz with position/orientation `pos`/`ori` (lists, often NumPy arrays) in the world frame `"world"` and with radius `rad`.

![](https://github.com/cmower/ros_helper/blob/master/doc/images/sphere.png?raw=true)

Normal implementation without using `ros_helper`.
```python
import rospy
from visualization_msgs.msg import Marker

rospy.init_node('example_node')
pos = [1, 2, 3]
rad = 0.5

pub = rospy.Publisher('sphere', Marker, queue_size=1)

def update(event):
  marker = Marker()
  marker.id = 0
  marker.header.frame_id = "world"
  marker.header.stamp = rospy.Time.now()
  marker.type = Marker.SPHERE
  marker.action = Marker.ADD
  marker.scale.x = 2 * rad
  marker.scale.y = 2 * rad
  marker.scale.z = 2 * rad
  marker.pose.position.x = pos[0]
  marker.pose.position.y = pos[1]
  marker.pose.position.z = pos[2]
  marker.pose.orientation.w = 1.0
  marker.color.r = 1.0
  marker.color.a = 1.0

  pub.publish(marker)

rospy.Timer(rospy.Duration(1.0 / 20.0), update)
rospy.spin()
```

Using `ros_helper` you can simplify the code to the following.
```python
#!/usr/bin/env python
import rospy
from ros_helper.simple_pub_sub import SimpleConstPublisher
import ros_helper.msgs.visualization as vis

rospy.init_node('example_node')
SimpleConstPublisher(rospy, 'sphere', 20, vis.SphereMsg(frame_id="world", position=[1, 2, 3], radius=0.5, rgba=[1, 0, 0, 1]))
rospy.spin()
```

## Todo

### Short term

* Update `keyboard_node` to use `simple_pub_sub`, perhaps consider moving away from pygame.
* Finish `experiments` module.
* Update examples, e.g. the scripts names, and make more.
* Cleaner initialization handling in `msgs`, like new additions to `visualization`.
* Some scripts seem to have irregular spacing. Check through and ensure code complies with [PEP 8](https://www.python.org/dev/peps/pep-0008/).
  * Consider using pep8ify, see [here](https://github.com/spulec/pep8ify).
* Un-hackify `publish_environment_node`. Further functionality?
* Clean interfaces in `simple_pub_sub`.

### Long term

* Move to Python 3
* Proper exception handling and colored logging to terminal, consider Python [`logging`](https://docs.python.org/2/library/logging.html). Also, could we log to `./logs` in catkin workspace? Is all this really worth it?
* Finish implementing all classes in `msgs`
* Documentation
  * Wiki
  * Function/classes
* Add Exotica helper functionality, e.g. simple manual initializer setup.
* A C++ version and helper functionality specific to C++.
* Unit-testing.

### "May never happen" term

* Move main functionality of `ros_helper` to C++ and wrap around with Python (e.g. Pybind).
* Make public (maybe).
