#!/usr/bin/env python

import numpy as np
from tf.transformations import *

class Pose(object):
    '''A general SE3 pose object. Uses [qx qy qz qw] standard.'''

    def __init__( self ):
        self.quaternion = np.array( [0, 0, 0, 1] )
        self.translation = np.array( [0, 0, 0] )

    @H.getter
    def H( self ):
        