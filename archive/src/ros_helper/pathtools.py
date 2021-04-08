import rospkg as rp

def rh_path():
    return pkg_path('ros_helper')

def meta_pkg_path(mpkg_name):
    return rp.RosStack().get_path(mpkg_name)

def pkg_path(pkg_name):
    return rp.RosPack().get_path(pkg_name)
