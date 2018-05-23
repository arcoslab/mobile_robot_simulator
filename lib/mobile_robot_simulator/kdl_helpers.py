# -*- coding: utf-8 -*-
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Alexis Maldonado Herrera <maldonad at cs.tum.edu>
#          Federico Ruiz Ugalde <memeruiz@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# This was taken from arcospyu, and adapted to Python3

from numpy import array, identity
import PyKDL


def frame_to_list(f):
    temp = []
    for i in range(3):
        for j in range(4):
            temp.append(f[i, j])
    temp += [0., 0., 0., 1.]
    return (temp)


def narray_to_kdlframe(narray):
    kdl_frame = PyKDL.Frame()
    for i in range(3):
        for j in range(3):
            kdl_frame.M[i, j] = narray[i, j]
        kdl_frame.p[i] = array[i, 3]

    return kdl_frame


def kdlframe_to_narray(kdl_frame):
    narray = identity(4)
    for i in range(3):
        for j in range(3):
            narray[i, j] = kdl_frame.M[i, j]
        narray[i, 3] = kdl_frame.p[i]

    return narray


def narray_to_kdltwist(narray):
    twist = PyKDL.Twist(
        PyKDL.Vector(narray[0], narray[1], narray[2]),
        PyKDL.Vector(narray[3], narray[4], narray[5]))
    return (twist)


def kdltwist_to_narray(kdl_twist):
    return (array([var for var in kdl_twist]))


def kdl_rot_mat_to_narray(kdl_rot_mat):
    rot = identity(3)
    for r in range(3):
        for c in range(3):
            rot[r, c] = kdl_rot_mat[r, c]
    return (rot)


def rot_vector_angle(vector, angle):
    kdl_vector = PyKDL.Vector(vector[0], vector[1], vector[2])
    return (kdl_rot_mat_to_narray(PyKDL.Rotation.Rot(kdl_vector, angle)))


def kdl_rot_vector_angle(vector, angle):
    kdl_vector = PyKDL.Vector(vector[0], vector[1], vector[2])
    return (PyKDL.Rotation.Rot(kdl_vector, angle))


def add_delta(frame, twist, dt):
    return (
        kdlframe_to_narray(
            PyKDL.addDelta(
                narray_to_kdlframe(frame), narray_to_kdltwist(twist), dt)))


def diff(frame1, frame2, dt):
    return (
        kdltwist_to_narray(
            PyKDL.diff(
                narray_to_kdlframe(frame1), narray_to_kdlframe(frame2), dt)))


def get_euler_zyx(rot_mat):
    """
    Get the euler angles from a rotation matrix
    """
    rot_mat_kdl = PyKDL.Rotation()
    for r in range(3):
        for c in range(3):
            rot_mat_kdl[r, c] = rot_mat[r, c]
    return (array(rot_mat_kdl.GetEulerZYX()))


def rpy_to_rot_matrix(roll_pitch_yaw):
    """
    Get matrix from roll pitch yaw
    """
    kdl_matrix = PyKDL.Rotation.RPY(
        roll_pitch_yaw[0], roll_pitch_yaw[1], roll_pitch_yaw[2])
    return (kdl_rot_mat_to_narray(kdl_matrix))
