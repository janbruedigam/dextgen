"""Rotations copy from the robot environments in OpenAI's gym.

Files have been removed from the current gym implementation.
"""
# Copyright (c) 2009-2017, Matthew Brett and Christoph Gohlke
#    All rights reserved.
#
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are
#    met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
#    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
#    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#    LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Many methods borrow heavily or entirely from transforms3d:
# https://github.com/matthew-brett/transforms3d
# They have mostly been modified to support batched operations.

import numpy as np
"""
Rotations
=========
Note: these have caused many subtle bugs in the past.
Be careful while updating these methods and while using them in clever ways.
See MuJoCo documentation here: http://mujoco.org/book/modeling.html#COrientation
Conventions
-----------
    - All functions accept batches as well as individual rotations
    - All rotation conventions match respective MuJoCo defaults
    - All angles are in radians
    - Matricies follow LR convention
    - Euler Angles are all relative with 'xyz' axes ordering
    - See specific representation for more information
Representations
---------------
Euler
    There are many euler angle frames -- here we will strive to use the default
        in MuJoCo, which is eulerseq='xyz'.
    This frame is a relative rotating frame, about x, y, and z axes in order.
        Relative rotating means that after we rotate about x, then we use the
        new (rotated) y, and the same for z.
Quaternions
    These are defined in terms of rotation (angle) about a unit vector (x, y, z)
    We use the following <q0, q1, q2, q3> convention:
            q0 = cos(angle / 2)
            q1 = sin(angle / 2) * x
            q2 = sin(angle / 2) * y
            q3 = sin(angle / 2) * z
        This is also sometimes called qw, qx, qy, qz.
    Note that quaternions are ambiguous, because we can represent a rotation by
        angle about vector <x, y, z> and -angle about vector <-x, -y, -z>.
        To choose between these, we pick "first nonzero positive", where we
        make the first nonzero element of the quaternion positive.
    This can result in mismatches if you're converting an quaternion that is not
        "first nonzero positive" to a different representation and back.
Axis Angle
    (Not currently implemented)
    These are very straightforward.  Rotation is angle about a unit vector.
XY Axes
    (Not currently implemented)
    We are given x axis and y axis, and z axis is cross product of x and y.
Z Axis
    This is NOT RECOMMENDED.  Defines a unit vector for the Z axis,
        but rotation about this axis is not well defined.
    Instead pick a fixed reference direction for another axis (e.g. X)
        and calculate the other (e.g. Y = Z cross-product X),
        then use XY Axes rotation instead.
SO3
    (Not currently implemented)
    While not supported by MuJoCo, this representation has a lot of nice features.
    We expect to add support for these in the future.
TODO / Missing
--------------
    - Rotation integration or derivatives (e.g. velocity conversions)
    - More representations (SO3, etc)
    - Random sampling (e.g. sample uniform random rotation)
    - Performance benchmarks/measurements
    - (Maybe) define everything as to/from matricies, for simplicity
"""

# For testing whether a number is close to zero
_FLOAT_EPS = np.finfo(np.float64).eps
_EPS4 = _FLOAT_EPS * 4.0


def mat2euler(mat: np.ndarray) -> np.ndarray:
    """Convert Rotation Matrix to Euler Angles.

    Args:
        mat: Rotation matrix.

    See rotation.py for notes
    """
    mat = np.asarray(mat, dtype=np.float64)
    assert mat.shape[-2:] == (3, 3), "Invalid shape matrix {}".format(mat)

    cy = np.sqrt(mat[..., 2, 2] * mat[..., 2, 2] + mat[..., 1, 2] * mat[..., 1, 2])
    condition = cy > _EPS4
    euler = np.empty(mat.shape[:-1], dtype=np.float64)
    euler[..., 2] = np.where(
        condition,
        -np.arctan2(mat[..., 0, 1], mat[..., 0, 0]),
        -np.arctan2(-mat[..., 1, 0], mat[..., 1, 1]),
    )
    euler[..., 1] = np.where(condition, -np.arctan2(-mat[..., 0, 2], cy),
                             -np.arctan2(-mat[..., 0, 2], cy))
    euler[..., 0] = np.where(condition, -np.arctan2(mat[..., 1, 2], mat[..., 2, 2]), 0.0)
    return euler
