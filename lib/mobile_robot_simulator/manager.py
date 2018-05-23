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

import yarp
import json
from multiprocessing import Process, Pipe
from time import time, sleep
from numpy import array

from differential_robot import DifferentialRobot


def manager_process(frequency, robot_indexes, robot_conf, point_obstacles_port,
                    line_obstacles_port,
                    goals_port, out_robots_port):

    robots = []
    poses = []
    for index in robot_indexes:
        initial_pose = array(robot_conf["poses"][index])
        initial_vel = array([0, 0, 0, 0, 0, 0])
        robots.append(DifferentialRobot(
            index, initial_pose, initial_vel, robot_conf["dim"],
            robot_conf["center"], robot_conf["wheel_radius"],
            robot_conf["wheel_distacne"], robot_conf["center_distance"],
            robot_conf["weight"], robot_conf["max_vel"]))
        poses.append(initial_pose)

    point_obstacles = poses
    line_obstacles = None
    goals = poses

    while True:
        init_time = time()
        if point_obstacles_port.poll():
            point_obstacles = point_obstacles_port.recv()
        if line_obstacles_port.poll():
            line_obstacles = line_obstacles_port.recv()
        if goals_port.poll():
            goals = goals_port.recv()

        poses = []
        for robot in robots:
            robot.update_goal(goals[robot.index])
            robot.update_obstacles(point_obstacle_list=point_obstacles,
                                   line_obstacle_list=line_obstacles)
            poses.append(robot.update_state(1/frequency))

        out_robots_port.send(poses)

        elapsed = time() - init_time
        assert 1/frequency - elapsed > 0, "Manager Process Cycle too slow"
        sleep(1/frequency - elapsed)


class Manager():
    def __init__(self, scene_conf):
        self.max_robots_per_process = scene_conf["max_robots_per_process"]
        self.robots_num = scene_conf["robots_num"]
        self.frequency = scene_conf["frequency"]
        self.robot_conf = scene_conf["robot_conf"]

        self.robot_poses = map(array, scene_conf["poses"])

        self._connect()

        self._create_processes()

    def _connect(self):
        local_goals_name = "/mobile_robot_simulator/manager/goals"
        remote_goals_name = "/mobile_robot_simulator/goals_obstacles/goals"
        self.goals_port = yarp.BufferedPortBottle()
        self.goals_port.open(local_goals_name)

        local_point_obs_name = (
            "/mobile_robot_simulator/manager/point_obstacles")
        remote_point_obs_name = (
            "/mobile_robot_simulator/goals_obstacles/point_obstacles")
        self.point_obstacles_port = yarp.BufferedPortBottle()
        self.point_obstacles_port.open(local_point_obs_name)

        local_line_obs_name = (
            "/mobile_robot_simulator/manager/line_obstacles")
        remote_line_obs_name = (
            "/mobile_robot_simulator/goals_obstacles/line_obstacles")
        self.line_obstacles_port = yarp.BufferedPortBottle()
        self.line_obstacles_port.open(local_line_obs_name)

        local_robots_name = (
            "/mobile_robot_simulator/manager/robots")
        remote_robots_name = (
            "/mobile_robot_simulator/vis/robots")
        self.robots_port = yarp.BufferedPortBottle()
        self.robots_port.open(local_robots_name)

        sleep(0.2)  # give some time to yarp
        yarp.Network.connect(remote_goals_name, local_goals_name,
                             carrier="shmem")
        yarp.Network.connect(remote_point_obs_name, local_point_obs_name,
                             carrier="shmem")
        yarp.Network.connect(remote_line_obs_name, local_line_obs_name,
                             carrier="shmem")
        yarp.Network.connect(local_robots_name, remote_robots_name,
                             carrier="shmem")
        sleep(0.2)  # Give some time to yarp

    def _create_processes(self):
        self.num_processes = int(self.robots_num / self.max_robots_per_process)
        if self.robots_num % self.max_robots_per_process != 0:
            self.num_processes += 1

        self.robots_per_process = []
        self.point_obstacles_pipes = []
        self.line_obstacles_pipes = []
        self.goals_pipes = []
        self.out_robots_pipes = []

        # Create Pipes
        for process_id in range(self.num_processes):
            self.point_obstacles_pipes.append(Pipe())
            self.line_obstacles_pipes.append(Pipe())
            self.goals_pipes.append(Pipe())
            self.out_robots_pipes.append(Pipe())
            self.robots_per_process.append([])

        # Create robots lists
        process_id = 0
        robots_in_process = 0
        for robot_index in range(self.robots_num):
            self.robots_per_process[process_id].append(robot_index)
            robots_in_process += 1
            if robots_in_process == self.max_robots_per_process:
                process_id += 1
                robots_in_process = 0

        # Create processes
        self.processes = []
        for process_id in range(self.num_processes):
            self.processes.append(
                Process(target=manager_process,
                        args=(self.frequency,
                              self.robots_per_process[process_id],
                              self.robot_conf,
                              self.point_obstacles_pipes[process_id][0],
                              self.line_obstacles_pipes[process_id][0],
                              self.goals_pipes[process_id][0],
                              self.out_robots_pipes[process_id][0]))
            )

    def _recv_goal_poses(self):
        btl = self.goals_port.read(False)
        if btl:
            str_msg = btl.get(0).asString()
            goal_poses = map(array, json.loads(str_msg))

            for goal_pipe in self.goals_pipes:
                goal_pipe[1].send(goal_poses)

    def _recv_obstacle_poses(self):
        btl = self.point_obstacles_port.read(False)
        if btl:
            str_msg = btl.get(0).asString()
            obs_poses = map(array, json.loads(str_msg))
            # Other robots are considered as punctual obstacles
            obs_poses = self.robot_poses + obs_poses

            for point_obs_pipe in self.point_obstacles_pipes:
                point_obs_pipe[1].send(obs_poses)

        btl = self.line_obstacles_port.read(False)
        if btl:
            str_msg = btl.get(0).asString()
            obs_poses = map(array, json.loads(str_msg))

            for line_obs_pipe in self.line_obstacles_pipes:
                line_obs_pipe[1].send(obs_poses)

    def _send_robot_poses(self):
        robot_poses_per_process = []
        for robot_pipe in self.out_robots_pipes:
            assert robot_pipe.poll(), "A process is not responding"
            robot_poses_per_process.append(robot_pipe.recv())

        self.robot_poses = []
        map(self.robot_poses.extend, robot_poses_per_process)

        btl = self.robots_port.prepare()
        str_msg = json.dumps(map(list, self.robot_poses))
        btl.clear()
        btl.addString(str_msg)
        self.robots_port.write()

    def loop(self):
        for process in self.processes:
            process.start()

        sleep(1/self.frequency)
        self.keep_alive = True

        while self.keep_alive:
            init_time = time()
            self._recv_goal_poses()
            self._recv_obstacle_poses()

            self._send_robot_poses()

            elapsed = time() - init_time
            assert 1/self.frequency - elapsed > 0, "Manager Cycle too slow"
            sleep(1/self.frequency - elapsed)

    def terminate(self):
        self.keep_alive = False
        for process in self.processes:
            process.terminate()
