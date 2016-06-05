#!/usr/bin/env python

import numpy as np
from argus_msgs.msg import MatrixFloat64

def MsgToMatrix( msg ):
    if len( msg.data != msg.rows * msg.cols ):
        raise RuntimeError( 'Message matrix dimensions do not match data.' )

    if msg.column_major:
        return np.reshape( msg.data, [msg.rows, msg.cols], order='F' )
    else:
        return np.reshape( msg.data, [msg.rows, msg.cols], order='C' )

def MatrixToMsg( mat ):
    if len(mat.shape) > 2:
        raise RuntimeError( 'Cannot convert array with more than 2 dimensions.' )

    msg = MatrixFloat64()
    msg.column_major = True;
    msg.rows = mat.shape[0]
    msg.cols = mat.shape[1]
    msg.data = mat.flatten( order='F' )
    return msg