#!/usr/bin/env python3
import sys
import rospy
import numpy
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):

        # Initialization
        RosNode.__init__(self, rospy)
        self.initNode('save_tf_node')

        # Get frames, assumes they are given by command line arguments
        self.base_frame_id = sys.argv[1]
        self.child_frame_id = sys.argv[2]
        if len(sys.argv) > 3:
            self.num_tfs_to_collect = int(sys.argv[3])
        else:
            self.num_tfs_to_collect = 200
        if len(sys.argv) > 4:
            self.timeout = int(sys.argv[4])
        else:
            self.timeout = 100
        self.n_attempts = 0
        self.tfs = []

        # Setup tf listener
        self.listenToTf('tf', self.base_frame_id, self.child_frame_id, frequency=150)

        # Start main loop
        rospy.loginfo('Logging tfs...')
        self.startTimer('main', 100, self.main)

    def main(self, event):

        # If nothing received yet return
        if not self.tfRetrieved('tf'):
            self.n_attempts += 1
            if self.n_passes == self.timeout:
                rospy.logerr(f'Did not hear any tfs in {self.timeout} attempts.')
                self.baseShutdown()
                sys.exit(1)
            return

        # Grab tf and append to tfs
        tf = self.retrieveTf('tf')
        p = self.positionFromTf2Msg(tf).tolist()
        q = self.quaterionFromTf2Msg(tf).tolist()
        self.tfs.append(p + q)

        # Report
        n = len(self.tfs)
        rospy.loginfo(f'Collected {n}/{self.num_tfs_to_collect} tfs.')

        # Check if collected enough
        if n == self.num_tfs_to_collect:
            rospy.loginfo('Complete.')

            # Compute tf
            tf = numpy.array(self.tfs).mean(axis=0)

            # Save tf
            stamp = self.uniqueTag()
            filename = f'tf_{self.base_frame_id}_{self.child_frame_id}_{stamp}.npy'
            numpy.save(filename, tf)
            rospy.loginfo('Saved: %s' % filename)

            # Shutdown
            self.baseShutdown()
            sys.exit(0)

if __name__ == '__main__':
    Node().spin()


