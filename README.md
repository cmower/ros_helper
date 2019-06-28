# ros_helper

A collection of helper code for [ROS](https://www.ros.org/). For instructions, explanations, and examples see the wiki [here](https://github.com/cmower/ros_helper/wiki) (unfinished!). The main reason this package exists is that writing packages/modules for ROS tends to lead to a lot of code repetition; this package attempts to address this issue. `ros_helper` also provides a number of other facilities that are "helpful"!

# Simple examples

## Publish spherical markers

Normal method for a typical example. This  assumes you want to publish a red marker with position/orientation `pos`/`ori` (numpy arrays) in the world frame `"world_frame"` and with radius `rad`.
```python
import rospy
from visualization_msgs.msg import Marker

pub = rospy.Publisher('sphere', Marker, queue_size=1)

... code ...

marker = Marker()
marker.id = 0
marker.header.stamp = rospy.Time.now()
marker.header.frame_id = "world_frame"
marker.type = Marker.SPHERE
marker.action = Marker.ADD
marker.scale.x = 2 * rad
marker.scale.y = 2 * rad
marker.scale.z = 2 * rad
marker.pose.position.x = pos[0]
marker.pose.position.y = pos[1]
marker.pose.position.z = pos[2]
marker.pose.orientation.x = ori[0]
marker.pose.orientation.y = ori[1]
marker.pose.orientation.z = ori[2]
marker.pose.orientation.w = ori[3]
marker.color.r = 1.0
marker.color.a = 1.0

pub.publish(marker)
```

Using `ros_helper` you can achieve the same task using the following.
```python
import rospy
import ros_helper.msgs.visualziation as vis

pub = rospy.Publisher('sphere', vis.SphereMsg, queue_size=1)

... code ...

pub.publish(vis.SphereMsg(time=rospy.Time.now(), frame_id="world_frame", positon=pos, orientation=ori, radius=rad, color=[1, 0, 0, 1]))
```

## Todo

### Short term 

* Update `keyboard_node` to use `simple_pub_sub`
* Proper exception handling and colored logging to terminal.
* Update other classes in `simple_pub_sub` to be aligned with the new updates to publisher.

### Long term

* Move to Python 3
* Documentation (Wiki).
* Add Exotica helper functionality, e.g. simple manual initializer setup. 
* A C++ version and helper functionality specific to C++. 
* Make public.

### "May never happen" term

* Move main functionality of `ros_helper` to C++ and wrap around with Python (e.g. Pybind).
