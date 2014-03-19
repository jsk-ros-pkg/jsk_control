# tf_ext.py
# several extention function for tf
import tf
from tf.transformations import *

def transformToMatrix(transform):
    # transform = (pos, quaternion)
    mat = quaternion_matrix(transform[1])
    mat[:3, 3] = transform[0]
    return mat

def xyzxyzwToMatrix(xyzxyzw):
    # convert [x, y, z, xx, yy, zz, ww] to matrix
    # where xx, yy, zz and ww mean orientation parameter.
    return transformToMatrix((xyzxyzw[:3], xyzxyzw[3:]))

def decomposeMatrix(mat):
    # convert matrix into (pos, quaternion) tuple
    pos = mat[:3, 3]
    q = quaternion_from_matrix(mat)
    return (pos, q)
