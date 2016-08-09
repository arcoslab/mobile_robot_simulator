#!/usr/bin/env python
# Copyright (c) 2016 Autonomous Robots and Cognitive Systems Laboratory. Universidad de Costa Rica
# Author: Daniel Garcia Vaglio degv364@gmail.com 
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

# handles obstacles and goals for basic simulations

#system
import sys, time
#numpy
from numpy import array, identity
from numpy.linalg import norm
from numpy.random import random
#arcospyu
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port
from arcospyu.rawkey.rawkey import Raw_key, Keys
from arcospyu.control.control_loop import Controlloop
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta

#press o for changing goal
#press q for changing obstacle
#press r for random obscale position
#press f for random obstacle movement
#use arrows to control goals
#use wasd to control obstacles

class Control(Controlloop):

    def set_params(self,params):
        #general initialization
        vars(self).update(params) #takes the dictionary params and puts everything in variables of the class
        self.raw_key=Raw_key()
        base_name="/goal_obstacle"

        #goal port and position  initialization
        self.goal_out_port_list=[]
        self.goal_pose_list=[0]*self.robot_cant
        self.goal_vel_list=[0]*self.robot_cant
        for i in range(self.robot_cant):
            self.goal_out_port_list.append(new_port(base_name+"/goal"+str(i)+":out","out", "/differential_robot/goal"+str(i)+":in", timeout=2))
            self.goal_pose_list[i]=identity(4)
            self.goal_pose_list[i][:3,3]=array([0.1, i, 0.])
            self.goal_vel_list[i]=array([0.,0.,0.,0.,0.,0.])

        #obstacle port position initialization
        self.obstacle_out_port_list=[]
        self.obstacle_pose_list=[0]*self.obstacle_cant
        self.obstacle_vel_list=[0]*self.obstacle_cant
        for i in range(self.obstacle_cant):
            self.obstacle_out_port_list.append(new_port(base_name+"/obstacle"+str(i)+":out","out", "/differential_robot/obstacle"+str(i)+":in", timeout=2))
            self.obstacle_pose_list[i]=identity(4)
            self.obstacle_vel_list[i]=array([0.,0.,0.,0.,0.,0.])

        
        #operation information
        self.random_movement=False
        self.robot_index=0
        self.obstacle_index=0
        self.last_time=time.time()
        self.inc_vel=0.43
        
        
    def process(self):
       
        #keys reading part
        chars=self.raw_key.get_num_chars()
        if len(chars)>0:
            #select obstacle
            if chars==Keys.q:
                self.obstacle_index+=1
                self.obstacle_index=self.obstacle_index%self.obstacle_cant
            #select robot goal
            if chars==Keys.o:
                self.robot_index+=1
                self.robot_index=self.robot_index%self.robot_cant
            #control goal
            if chars==Keys.UP_ARROW:
                self.goal_vel_list[self.robot_index][1]+=self.inc_vel
            if chars==Keys.DOWN_ARROW:
                self.goal_vel_list[self.robot_index][1]-=self.inc_vel
            if chars==Keys.RIGHT_ARROW:
                self.goal_vel_list[self.robot_index][0]+=self.inc_vel
            if chars==Keys.LEFT_ARROW:
                self.goal_vel_list[self.robot_index][0]-=self.inc_vel
            #control obstacle
            if chars==Keys.w:
                self.obstacle_vel_list[self.obstacle_index][1]+=self.inc_vel
            if chars==Keys.s:
                self.obstacle_vel_list[self.obstacle_index][1]-=self.inc_vel
            if chars==Keys.a:
                self.obstacle_vel_list[self.obstacle_index][0]-=self.inc_vel
            if chars==Keys.d:
                self.obstacle_vel_list[self.obstacle_index][0]+=self.inc_vel
            #positions and movement options
            if chars==Keys.r:
                #give every obstacle a random position
                for i in range(self.obstacle_cant):
                    self.obstacle_vel_list[i]=array([0.,0.,0.,0.,0.,0.])
                    self.obstacle_pose_list[i][:2,3]=10*random(2)-array([5,5])
            if chars==Keys.f:
                #give every obstacle a random movement behaviour
                self.random_movement=not self.random_movement
                for i in range(self.obstacle_cant):
                    self.obstacle_vel_list[i][:2]=0.*random(2)-array([0.025,0.025])

        #time meassurement
        self.cur_time=time.time()
        dt=self.cur_time-self.last_time
        self.last_time=self.cur_time

        #make obstacles move randomly if necesary
        if self.random_movement:
            for i in range(self.obstacle_cant):
                self.obstacle_vel_list[i][:2]-=0.1*self.obstacle_vel_list[i][:2]
                self.obstacle_vel_list[i][:2]+=0.6*random(2)-array([0.3,0.3])
                if norm(self.obstacle_pose_list[i][:2,3])>12:
                    self.obstacle_vel_list[i][:2]+=-self.obstacle_pose_list[i][:2,3]/12.0


        #send information throught yarp
        for i in range(self.robot_cant):
            self.goal_pose_list[i]=my_adddelta(self.goal_pose_list[i],self.goal_vel_list[i],dt)
            write_narray_port(self.goal_out_port_list[i],self.goal_pose_list[i][:3,3])
        for i in range(self.obstacle_cant):
            self.obstacle_pose_list[i]=my_adddelta(self.obstacle_pose_list[i],self.obstacle_vel_list[i],dt)
            write_narray_port(self.obstacle_out_port_list[i], self.obstacle_pose_list[i][:3,3])


        
def main():

    obstacle_cant=15
    robot_cant=3
    control_loop=Control(15.)
    
    control_loop.loop(obstacle_cant=obstacle_cant, robot_cant=robot_cant)
    

if __name__=="__main__":
    main()


