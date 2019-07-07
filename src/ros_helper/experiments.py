import os
import inspect
import pandas as pd
import numpy as np
from .utils import *

"""

 >>> unfinished <<<

inspired by my_container_base.py and results.py in experiment_comparison_of_formulations_for_spraying_task 
see here: https://github.com/cmower/experiment_comparison_of_formulations_for_spraying_task/tree/master

"""

print "[WARN] ros_helper.experiments is still in development, do not use!"

#
# Base packet class and some common inherited classes
#

class Packet(object):

    def __init__(self, col_names, ndata, dtype):

        # Input check
        is_array_str(col)
        is_strictly_positive_int(ndata)

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
        
        # Input check
        is_int(i)
        in_int_range(i, self.ndata)
        is_array_dtype(v, self.__dtype)
        if len(v) is not self.ncols:
            raise TypeError("expected data length of %d, for %d" % (self.ndata, len(v)))

        # Assign data
        self.__data[i,:] = np.asarray(v, dtype=self.__dtype)

    def __getitem__(self, i):

        # Input check
        is_int(i)
        in_int_range(i, self.ndata)

        return self.__data[i,:]

    def to_dataframe(self):
        return pd.DataFrame({col_name : self.__data[:,i] \
                             for i, col_name in enumerate(self.__col_names)},\
                            dtype=self.__dtype,\
                            index=np.arange(0, self.ndata))

class KukaLWRJointTrajectory(Packet):

    def __init__(self, name, ndata):

        # Input check
        is_str(name)
        is_strictly_positive_int(ndata)

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

        # Input check
        is_str(name)
        is_strictly_positive_int(ndata)

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

        # Input check
        is_dict_keys_str(init)
        for v in init.values():
            if Packet not in inspect.getmro(type(v)):
                raise TypeError("Must inherit from Packet")
        is_strictly_positive_int(ndata)
        is_dir(path2data)

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
