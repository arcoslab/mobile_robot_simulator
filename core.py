#!/usr/bin/env python
# Copyright (c) 2011 Autonomous Robots and Cognitive Systems Laboratory
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

# This program was developed by Daniel Garcia Vaglio (degv364@gmail.com).
# Inspired from a program by Federico Ruiz.

# This is a program that will simulate a differential robot.

# system
from builtins import str
from builtins import range
import time
# numpy
from numpy import array, identity, dot, pi
from numpy.linalg import norm
from numpy.random import normal
# pyrovito
from pyrovito.pyrovito_utils import Roboviewer_objects
# arcospyu
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta
from arcospyu.control.control_loop import Controlloop
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort
from arcospyu.robot_tools.robot_trans import homo_matrix
from arcospyu.rawkey.rawkey import Raw_key, Keys
# imports from this project
from mobile_robot_simulator.vectorfield.points import vectorfield
from mobile_robot_simulator.robot_models import Differential_robot


class Loop(Controlloop):
    def visualize(self, points=[], vectors=[]):
        for local, ref_frame, vis_id in points:
            if len(local) == 3:
                # for points
                pose = dot(ref_frame, homo_matrix(trans=local))
            else:
                # for frames
                pose = dot(ref_frame, local)
            self.view_objects.send_prop(vis_id, "pose",
                                        pose.reshape(16).tolist())

        for single_vector in vectors:
            vector_local = single_vector[0]
            ref_frame_vector = single_vector[1]
            point_local = single_vector[2]
            ref_frame_point = single_vector[3]
            vis_id = single_vector[4]
            point = dot(ref_frame_point, homo_matrix(trans=point_local))
            self.view_objects.send_prop(vis_id, "pose",
                                        point.reshape(16).tolist())
            vector = dot(ref_frame_vector[:3, :3], vector_local)
            self.view_objects.send_prop(vis_id, "axis",
                                        0.5 * vector / norm(vector))

    def set_params(self, params):
        # General initialization takes the dictionary params and puts
        # everything in variables of the class
        vars(self).update(
            params
        )
        base_name = "/differential_robot"

        # port initialization
        self.goal_in_port_list = []
        for i in range(self.robot_cant):
            self.goal_in_port_list.append(
                new_port(
                    base_name + "/goal" + str(i) + ":in",
                    "in",
                    "/goal_obstacle/goal" + str(i) + ":out",
                    timeout=1.))
        self.obstacle_in_port_list = []
        for i in range(self.obstacle_cant):
            self.obstacle_in_port_list.append(
                new_port(
                    base_name + "/obstacle" + str(i) + ":in",
                    "in",
                    "/goal_obstacle/obstacle" + str(i) + ":out",
                    timeout=1.))

        self.last_time = time.time()

        # differential_robots
        self.robot_list = []

        # variables for each robot

        self.robot_pose_list = []
        self.robot_vel_twist_list = []
        self.robot_pose_out_list = []
        self.phi_l_list = []
        self.phi_r_list = []
        self.angle_list = []
        self.omega_list = []
        self.required_velocity_list = []
        self.goal_pose_list = []
        robot_id_offset = identity(4)

        for i in range(self.robot_cant):
            # initialization
            if self.robot_type_list[i] == "differential":
                self.robot_list.append(Differential_robot(
                ))  # this line defines what robot is going to be used
            else:
                self.robot_list.append(Differential_robot(
                ))  # this line defines what robot is going to be used

            self.robot_pose_list.append(identity(4))
            self.robot_pose_list[i][:3, 3] = array([1.85, i, 0.])
            self.robot_vel_twist_list.append(array([0., 0., 0., 0., 0., 0.]))
            self.robot_pose_out_list.append(array(self.robot_pose_list[i]))
            self.phi_l_list.append(0.0)
            self.phi_r_list.append(0.0)
            self.angle_list.append(0.0)
            self.omega_list.append(0.0)
            self.required_velocity_list.append(array([0., 0., 0.]))
            self.goal_pose_list.append(identity(4))
            # visualization
            self.view_objects.send_prop(self.robot_id_list[i], "scale",
                                        self.robot_list[i].robot_dim)
            self.view_objects.send_prop(self.robot_id_list[i], "pose_offset",
                                        robot_id_offset.reshape(16).tolist())

        self.raw_key = Raw_key()

        # choose a set of equations
        self.move = False
        self.goal_constant = 1.
        self.obstacle_constant = 1.
        self.damp_constant = 0.5
        self.obstacle_pose_list = [1] * self.obstacle_cant
        for i in range(self.obstacle_cant):
            self.obstacle_pose_list[i] = identity(4)

    def process(self):
        vis_vectors = []
        vis_points = []

        # time
        self.cur_time = time.time()
        dt = self.cur_time - self.last_time
        self.last_time = self.cur_time

        # press < m > to start or end robot motion
        chars = self.raw_key.get_num_chars()
        if len(chars) > 0:
            if chars == Keys.m:
                self.move = not (self.move)

        # creat temporal variables to have valid inputs
        goal_pos_list = [0] * self.robot_cant
        robot_pos_list = [0] * self.robot_cant
        for i in range(self.robot_cant):
            goal_pos_list[i] = self.goal_pose_list[i][:3, 3]
            robot_pos_list[i] = self.robot_pose_list[i][:3, 3]
        obs_pose_list = [0] * self.obstacle_cant
        for i in range(self.obstacle_cant):
            obs_pose_list[i] = self.obstacle_pose_list[i][:3, 3]

        if self.move:
            for i in range(self.robot_cant):
                # include the other robots as obstacles
                robots_as_obstacles = robot_pos_list[:]
                robots_as_obstacles.pop(i)
                self.required_velocity_list[i] = vectorfield(
                    goal_pos_list[i], obs_pose_list + robots_as_obstacles,
                    robot_pos_list[i], self.required_velocity_list[i], dt,
                    self.goal_constant, self.obstacle_constant,
                    self.damp_constant)

                self.angle_list[i] += self.robot_vel_twist_list[i][5] * dt
                self.robot_vel_twist_list[
                    i] = self.robot_list[i].central_velocity(
                        self.required_velocity_list[i], self.angle_list[i])

        else:
            # in this case move=False, then velocity=0
            for i in range(self.robot_cant):
                self.robot_vel_twist_list[i] = array([0., 0., 0., 0., 0., 0.])

        # receive from yarp goal positions
        for i in range(self.robot_cant):
            goal_result = readListPort(
                self.goal_in_port_list[i], blocking=False)
            if goal_result:
                self.goal_pose_list[i][:3, 3] = array(goal_result)

            # robot visualization, and goal visualization
            vis_points += [(self.robot_pose_out_list[i], identity(4),
                            self.robot_id_list[i])]
            vis_points += [(self.robot_pose_out_list[i], identity(4),
                            self.front_id_list[i])]
            vis_points += [(self.goal_pose_list[i], identity(4),
                            self.goal_id_list[i])]

        # receive from yarp obstacle positions
        obstacle_result = [0.] * self.obstacle_cant
        for i in range(self.obstacle_cant):
            obstacle_result[i] = readListPort(
                self.obstacle_in_port_list[i], blocking=False)
            if obstacle_result[i]:
                # obstacle visualization
                self.obstacle_pose_list[i][:3, 3] = array(obstacle_result[i])
                vis_points += [(self.obstacle_pose_list[i], identity(4),
                                self.obstacle_id_list[i])]

        # visualization update
        self.visualize(points=vis_points, vectors=vis_vectors)

        # position update
        for i in range(self.robot_cant):
            self.robot_pose_list[i] = my_adddelta(
                self.robot_pose_list[i], self.robot_vel_twist_list[i], dt)

            # gaussian noise to output information.
            self.robot_pose_out_list[i] = array(self.robot_pose_list[i])
            noise_level = 0.1 * 0.3 * array([0.02, 0.02, 5.0 * pi / 180.0])
            self.robot_pose_out_list[i][:2, 3] = normal(
                loc=self.robot_pose_out_list[i][:2, 3],
                scale=noise_level[:2],
                size=(2))
            random_z_angle = normal(loc=0., scale=noise_level[2])
            surface_object_normal = -array(
                self.robot_list[i].robot_planes[self.robot_list[i]
                                                .surface_object_face][0])
            random_rot_frame = identity(4)
            random_rot_frame[:2, 3] = 0 * array([0.0, 0.01
                                                 ])  # camera offset error
            random_rot_frame[:3, :3] = rot_vector_angle(
                surface_object_normal, random_z_angle)
            self.robot_pose_out_list[i] = dot(self.robot_pose_out_list[i],
                                              random_rot_frame)


def main():
    # initialization of robots and goals
    view_objects = Roboviewer_objects("/planar_slider", "/lwr/roboviewer",
                                      2000)
    obstacle_cant = 15
    robot_cant = 3
    robot_type_list = ["differential"] * robot_cant
    # initialization of robots and goals
    robot_id_list = []
    front_id_list = []
    goal_id_list = []
    direction_offset = array([[0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0.1],
                              [0, 0, 0, 1]])
    for i in range(robot_cant):
        robot_id_list.append(view_objects.create_object("box"))
        view_objects.send_prop(robot_id_list[i], "scale", [1., 1., 1.])
        view_objects.send_prop(robot_id_list[i], "timeout", [-1])
        view_objects.send_prop(robot_id_list[i], "color", [0.5, 0.5, 0.5])

        # direction vector in front of each robot.
        front_id_list.append(view_objects.create_object("arrow"))
        view_objects.send_prop(front_id_list[i], "scale", [1, 1, 1])
        view_objects.send_prop(front_id_list[i], "pose_offset",
                               direction_offset.reshape(16).tolist())
        view_objects.send_prop(front_id_list[i], "timeout", [-1])
        view_objects.send_prop(front_id_list[i], "color", [1, 1, 0])

        # goal list, big green spheres
        goal_id_list.append(view_objects.create_object("sphere"))
        view_objects.send_prop(goal_id_list[i], "scale", [0.1, 0.1, 0.1])
        view_objects.send_prop(goal_id_list[i], "timeout", [-1])
        view_objects.send_prop(goal_id_list[i], "color", [0, 1, 0])

    # obstacle_list,  big red spheres
    obstacle_id_list = []

    for i in range(obstacle_cant):
        obstacle_id_list.append(view_objects.create_object("sphere"))
        view_objects.send_prop(obstacle_id_list[i], "scale", [0.1, 0.1, 0.1])
        view_objects.send_prop(obstacle_id_list[i], "timeout", [-1])
        view_objects.send_prop(obstacle_id_list[i], "color", [1, 0, 0])

    control_loop = Loop(15)
    control_loop.loop(
        view_objects=view_objects,
        robot_id_list=robot_id_list,
        obstacle_id_list=obstacle_id_list,
        front_id_list=front_id_list,
        goal_id_list=goal_id_list,
        obstacle_cant=obstacle_cant,
        robot_cant=robot_cant,
        robot_type_list=robot_type_list)


if __name__ == "__main__":
    main()
