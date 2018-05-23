# -*- coding: utf-8 -*-

# Copyright (c) 2016-2018 Autonomous Robots and Cognitive Systems Laboratory
# Universidad de Costa Rica
# Authors: Daniel Garcia Vaglio degv364@gmail.com
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

# This script takes the position of the goal, and the position of the
# obstacles and computes the direction a robot in a particular point needs.

from enum import Enum
from numpy import array
from numpy.linalg import norm, dot


class VectorFieldType(Enum):
    POINT = 0
    LINE = 1


def _point_vector_field(entity_pose, obstacle_pose, obstacle_constant=1):
    """
    Compute the 'force' that entity feels because of punctual obstacle

    :param entity_pose: 3D vector with the pos of an enity
    :param obstacle_pose: 3D vector with the pos of another entity
    :param obstacle_constant: repulsion of this obstacle

    :returns: 3D vector with contribution of obstacle to vectorfield
    """

    distance = norm(entity_pose[:3, 3] - obstacle_pose[:3, 3])
    force = (obstacle_constant * (entity_pose[:3, 3] - obstacle_pose[:3, 3]) /
             (distance**3))
    return force


def _line_vector_field(entity_pose, line_vertices, obstacle_constant=1):
    """
    Compute the 'force' that entity feels because of lineal obstacle

    :param entity_pose: 3D vector with the pos of an enity
    :param line_vertices: tuple with the vertices of the line
    :param obstacle_constant: repulsion of this obstacle

    :returns: 3D vector with contribution of obstacle to vectorfield
    """

    entity_rel = entity_pose[:3, 3] - line_vertices[0][:3, 3]
    line_rel = line_vertices[1][:3, 3] - line_vertices[0][:3, 3]
    proyection = ((dot(line_rel, entity_rel) /
                   (norm(entity_rel) * norm(line_rel))) * line_rel)

    if dot(proyection, line_rel) < 0:
        vector = entity_rel
    elif norm(proyection) > norm(line_rel):
        vector = entity_rel - line_rel
    else:
        vector = entity_rel - proyection

    force = obstacle_constant * vector / (norm(vector)**3)
    return force


def _multiple_vector_field(entity_pose, obstacle_list, obstacle_constants,
                           vector_field_type):

    vf_functions = {
        VectorFieldType.POINT: _point_vector_field,
        VectorFieldType.LINE: _line_vector_field
    }

    # Check correct lengths
    assert (obstacle_constants is None) or \
        (len(obstacle_constants) == len(obstacle_list), (
            "There is not the same number of point obstacles and constants"))

    force = array[0, 0, 0]
    # Compute the point force
    if obstacle_list is not None:
        if obstacle_constants is not None:
            force = force + sum(
                map(vf_functions[vector_field_type],
                    [entity_pose] * len(obstacle_list), obstacle_list,
                    obstacle_constants))
        else:
            force = force + sum(
                map(vf_functions[vector_field_type],
                    [entity_pose] * len(obstacle_list), obstacle_list))

    return force


def vector_field(dt,
                 entity_pose,
                 entity_vel,
                 goal_pose,
                 point_obstacle_list=None,
                 line_obstacle_list=None,
                 point_obstacle_constants=None,
                 line_obstacle_constants=None,
                 goal_constant=1,
                 damp_constant=0.5,
                 allowed_error=0.1):
    """
    Compute the required velocity based on the vectorfield

    :param dt: time differential
    :param entity_pos: 3D vector with entity position
    :param entity_vel: 3D vector with entity current velocity
    :param goal_pos: 3D vector with goal position
    :param point_obstacle_list: list with point obstacle positions
    :param line_obstacle_list: list with line obstacle vertices
    :param point_obstacle_constants: list with point repulsion factor
    :param line_obstacle_constants: list with line repulsion factor
    :param goal_constant: goal attraction factor
    :param damp_constant: damp entity velocity
    :param allowed_error: distance from goal at which entity should stop
    """

    # Stop when entity is very close to the goal
    if norm(entity_pose[:3, 3] - goal_pose[:3, 3]) < allowed_error:
        return array([0, 0, 0])

    force = array[0, 0, 0]
    # Point obstacles contribution
    force = force + _multiple_vector_field(entity_pose, point_obstacle_list,
                                           point_obstacle_constants,
                                           VectorFieldType.POINT)
    # Line obstacles contribution
    force = force + _multiple_vector_field(entity_pose, line_obstacle_list,
                                           line_obstacle_constants,
                                           VectorFieldType.LINE)
    # Goal contribution
    force = force + goal_constant * (goal_pose[:3, 3] - entity_pose[:3, 3])

    velocity = entity_vel + force * dt - (damp_constant * entity_vel)

    # Add rotational null velocity before returning
    return array([velocity[0], velocity[1], velocity[2], 0, 0, 0])
