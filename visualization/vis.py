#!/usr/bin/env python
# Copyright (c) 2017 ARCOS-lab Universidad de Costa Rica
# Author: Daniel Garcia Vaglio <degv364@gmail.com>
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

'''
 This si a visualization tool that uses multiprocessing isntead of YARP
 The way this is interacting with the rest of the system is by recieving a
 message with a dictionary. THe dictionary has the object ID, and the 
 attributes that must change. If a attribute is None, then it is not updated
 If an ID is not received, the entire object is not updated. If a non existing 
 ID is received, the object is created. The special isntruction delete, deletes
 the object. The attributes are a dictionary.
'''
import sys
import time

from multiprocessing import Connection
from threading import Thread

import pygame

from avispy.objects_lib import Arrow, Sphere, Disk, Bar, Frame, Cylinder, Directed_bar
from avispy.engine import Camera,Display,Colors,Scene, Light
from numpy import array, identity

#reads the obj_type string and creates the appropiate object        
def type_selector(obj_type):
    if obj_type == "arrow":    return Arrow()
    if obj_type == "shpere":   return Sphere()
    if obj_type == "box":      return Bar()
    if obj_type == "cylinder": return Cylinder()
    if obj_type == "frame":    return Frame()
    #Return directed bar by default
    return Directed_bar()
    
class Vis (object):
    def __init__(self, in_port):
        self.port    = in_port
        self.camera  = Camera()
        self.scene   = Scene()
        self.display = Display(self.camera, self.scene, res = (640,480))

        #set lights
        light0=Light(Light.LIGHTS[0])
        light0.position=array([10.,10.,10.,1.0])
        self.scene.add_light(light0)

        light1=Light(Light.LIGHTS[1])
        light1.position=array([-10.,10.,10.,1.0])
        self.scene.add_light(light1)
        
        light2=Light(Light.LIGHTS[2])
        light2.position=array([0.,-10.,10.,1.0])
        self.scene.add_light(light2)
        
        light3=Light(Light.LIGHTS[3])
        light3.position=array([0.,0.,-10.,1.0])
        self.scene.add_light(light3)

        #world frame
        world_frame=Frame()
        world_frame.scale=array([0.3,0.3,0.3])
        self.scene.add_object(world_frame)

        #dynamic objects
        self.object_dict = {"light0":light0, "light1":light1, "light2":light2
                            "light3":light3, "world_frame":world_frame}

    '''
    Create new object. If it alrady exist, update its info
    return true if it was created, false if it already existed
    '''
    def add_object(self, obj_id="ID:00",  obj_type="car",
                   obj_pose=identity(4), obj_scale = [1.,1.,1.],
                   obj_color= [1.,1.,1.]):
        was_created = False
        if obj_id not in self.obj_dict:
            was_created = True
            self.obj_dict[obj_id]=type_selector(obj_pose)

        self.update_object(obj_id, obj_pose, obj_scale, obj_color)
        return was_created
    '''
    Destroys an object. If it was already destroyed do nothing
    returns true it something was destoyed, false otherwise
    '''
    def remove_object(self, obj_id="ID:00"):
        #Check if I need to remove from scene object
        if object_id in self.object_dict:
            del self.object_dict[obj_id]
            return True
        return False
    '''
    Updates attributes of a given object.
    obj_pose is a homo matrix 4x4
    obj_scale is a list with 3 elements
    obj_color is a numpy array of 3 elements
    returns true if the object was found
    '''
    def update_object(self, obj_id="ID:00", obj_pose=None,
                      obj_scale=None, obj_color=None):
        if obj_id not is self.object_dict:
            return False
        if obj_pose is not None:
            self._update_pose(obj_id, obj_pose)
        if obj_scale is not None:
            self._update_scale(obj_id, obj_scale)
        if obj_color is not None:
            self._update_color(obj_id, obj_color)
        return True

    def _update_pose(self, obj_id, obj_pose):
        self.object_dict[obj_id].set_trans_rot_matrix(obj_pose)
    def _update_scale(self, obj_id, obj_scale):
        self.object_dict[obj_id].set_scale(obj_scale)
    def _update_color(self, obj_id, obj_color):
        self.object_dict[obj_id].set_color(obj_color)

    '''
    Update the camera from a pygame event
    '''
    def update_camera(self, pygame_event):
        self.camere.update(pygame_event)

    '''
    Update display
    '''
    def update(self):
        self.display.update()
        
    '''
    Reads the instruction dictionary from the in_port
    '''
    def read_instruction_dict(self):
        #lock execution until message is received
        instruction_dict = self.port.recv()
        for obj_id, attribute_dict in instruction_dict:
            obj_type = attribute_dict["obj_type"];
            obj_pose = attribute_dict["obj_pose"];
            obj_scale = attribute_dict["obj_scale"]
            obj_color = attribute_dict["obj_color"]
            delete = attribute_dict["delete"]
            
            if obj_id not in self.object_dict:
                self.add_object(obj_id, obj_type=obj_type,
                                obj_pose=obj_pose, obj_scale=obj_scale,
                                obj_color=obj_color)
            else:
                self.update_object(obj_id, obj_pose=obj_pose,
                                   obj_scale=obj_scale, obj_color=obj_color)
                if delete:
                    self.remove_object(obj_id)

def listen_for_events(args):
    vis_obj=args[0]
    continue_exec = args[1]
    while continue_exec:
        time.sleep(0.01)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type ==pygame.KEYDOWN and event.key == pygame.K_q):
                return None
            if event.type == pygame.MOUSEMOTION or event.type ==pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
                vis_obj.update_camera(event)
            
        
def visualization_process(in_port, control_port):
    continue_exec = True
    running = True
    vis_obj = Vis(in_port)
    listener = Thread(group=None,target=listen_for_events,name="Listen_events",
                      args=[vis_obj, continue_exec])
    listener.start()
    while continue_exec:
        continue_exec = listener.is_alive()
        if contol_port.poll():
            instruction = control_port.recv()
            if instruction=="termiante":
                continue_exec = False
            if instruction=="stall":
                print "Visualization process stall"
                running = False
            if instruction=="resume":
                print "Resuming visualization"
                running=True
        if running:
            vis_obj.read_instructions_dict()
            vis_obj.update()
        else:
            time.sleep(0.01)

    print "Terminating visualization process"
    if listener.is_alive():
        #FIXME kill the thread
        pass
    in_port.close()
    control_port.close()
    del vis_obj
    return None
    
        
        

if __name__=="__main__":
    dummy_main();
