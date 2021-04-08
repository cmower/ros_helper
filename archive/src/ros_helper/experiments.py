import os
import time
import inspect
import rosbag 
import pandas as pd
import numpy as np
from collections import deque
from functools import wraps
from .utils import *

"""

 >>> unfinished <<<

inspired by my_container_base.py and results.py in experiment_comparison_of_formulations_for_spraying_task 
see here: https://github.com/cmower/experiment_comparison_of_formulations_for_spraying_task/tree/master

"""

raise NotImplementedError("ros_helper.experiments is still in development, do not use!")

def rosbag2pandas(filename):
    pass

class Timer(object):

    """
    Timer 

    Timer decorator class for timing functions and collecting the results.

    Example
    -------
      Assume you have a function called solve, and you want to time how long it takes to
      complete. Use the timer class to easily gather this data as follows.

      : ---
      solver_timer = Timer()
      
      @solver_timer.timeit
      def solve():
          # code that you want to time

      #
      # Perform solve as many times as you like
      #
      for _ in xrange(20):
          solve()

      print solver_timer.Average
      : ---

      The output will be the average solver time.
    """

    def __init__(self):
        self.__data = deque([])

    def timeit(self, func):

        @wraps(func)
        def wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            end = time.time()
            self.__data.append(end - start)
            return result
        
        return wrapper

    @property
    def Data(self):
        return np.asarray(self.__data)

    @property
    def Average(self):
        return self.Data.mean()

#
# Base packet class and some common inherited classes
#

class Packet(object):

    def __init__(self, col_names, ndata, dtype):

        # Assign parameters
        self.col_names = col_names
        self.__dtype = dtype
        self.__data = np.empty((ndata, len(col_names)), dtype=dtype)

    @property
    def ndata(self):
        return self.__data.shape[0]

    @property
    def ncols(self):
        return self.__data.shape[1]

    def __setitem__(self, i, v):
    
        # Assign data
        self.__data[i,:] = np.asarray(v, dtype=self.__dtype)

    def __getitem__(self, i):


        return self.__data[i,:]

    def to_dataframe(self):
        return pd.DataFrame({col_name : self.__data[:,i] \
                             for i, col_name in enumerate(self.__col_names)},\
                            dtype=self.__dtype,\
                            index=np.arange(0, self.ndata))

class KukaLWRJointTrajectory(Packet):

    def __init__(self, name, ndata):

        # Setup packet                               
        if len(name) is 0:
            name_prefix = 'q'
        else:
            name_prefix = '%s q' % name
        
        super(KukaLWRJointTrajectory, self).__init__([\
                                                      '%s%d' % (name_prefix, i)\
                                                      for i in xrange(7)],\
                                                     ndata,\
                                                     float)
        
class Position3Trajectory(Packet):

    def __init__(self, name, ndata):

        # Setup packet
        if len(name) is 0:
            name_prefix = ''
        else:
            name_prefix = '%s ' % name
            
        super(Position3Trajectory,self).__init__(['%s%s' % (name_prefix, dim) for dim in XYZ],\
                                                 ndata,\
                                                 float)

#
# Container class
#

class Container(object):

    def __init__(self, name, init, ndata, path2data):

        # Setup
        self.name = name
        self.__path = path2data
        self.__container = init

    def __parsei(self, i):
        return i[0], i[1]
        
    def __setitem__(self, i, v):
        packet_name, idx = self.__parsei(i)
        self.__container[packet_name][idx]

    def __getitem__(self, i):
        packet_name, idx = self.__parsei(i)
        return self.__container[packet_name][idx]

    def save(self, packet_names=None):

        # Input check
        if packet_names is None:
            # save all
            packet_names_ = [name for name in self.__container.keys()]
        elif is_array_str(packet_names):
            # these packets
            packet_names_ = packet_names

        # Save
        pd.concat([self.__container[name].to_dataframe() for name in packet_names_],\
                  axis=1)
