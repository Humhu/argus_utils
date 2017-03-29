#!/usr/bin/env python

from tf.transformations import *
from geometry_msgs.msg import Pose

def MsgToPose( msg ):
    '''Converts a geometry_msgs.Pose message to a 