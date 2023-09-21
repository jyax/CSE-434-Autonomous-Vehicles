#!/usr/bin/env python
'''
    robot_room.py 

    This is a utility class that takes care of drawing a room, drawing a robot, 
    listening to the keyboard for commands, and moving the robot based on commands 
    and its room location.    No need to change this code.

    Note: This uses OpenCV which is not thread safe.  That means a node that uses this
    must keep all references to it in the same thread.  When you have callbacks, all
    references to this must either be in the callback or outside the callback, not both.
'''
import numpy as np
import cv2

class RobotRoom():


    def __init__(self, name, color=(255,255,255),  radius=10, boundary=10, speed = 4, sz=(320,240) ):
        ''' Initilaize RobotRoom: create an room with boundaries, and a robot in the center '''
        self.name = name
        self.color = color
        self.xy = np.array(sz) // 2   #starting location
        self.room = np.zeros((sz[1],sz[0],3),dtype=np.uint8)
        self.room[boundary:-boundary,boundary:-boundary,:] = 128  # gray boundary
        self.room[0:sz[1]//4*3, sz[0]//3*2:(sz[0]//3*2+boundary), :] = 0
        self.radius = radius
        self.speed = min(boundary-1,speed)
        self.draw_no_return_key()  # Let's draw the initial state
        cv2.moveWindow(self.name, 0, 0)

    def draw(self, wait=100):
        ''' Draw the location of the robot in the room
            And return the key pressed by the user.  
            Useful for leaders.
            wait: time in ms to wait for the user to press a key
        '''
        img = self.room.copy()
        cv2.circle( img, tuple(self.xy), self.radius, self.color, 4)  #Robot is a circle
        cv2.imshow(self.name, img)
        return cv2.waitKey(wait) & 0xFF

    def draw_no_return_key(self):
        ''' Draw the location of the robot and do not return a key 
            This is useful for followers
        '''
        self.draw(2)

    def listKeys(self):
        print('Keys: "a" left, "d" right, "w" up, "s" down, "q" quit')

    def move(self, key):
        ''' Updates position of the robot (self.xy) based on the input key and the speed
            The keys are integer values corresponding to keystrokes defined in listKeys()
            Also, the robot will not move onto a pixel location where the image is all-zero
              -- this prevents it leaving the room
        '''
        if key == ord('a'):
            nxy = self.xy + np.array([-1,0]) * self.speed
        elif key == ord('d'):
            nxy = self.xy + np.array([1,0]) * self.speed
        elif key == ord('w'):
            nxy = self.xy + np.array([0,-1]) * self.speed
        elif key == ord('s'):
            nxy = self.xy + np.array([0,1]) * self.speed
        else:
            return

        # Boundary pixels are black, so make sure we are not moving onto the boundary
        if self.room[nxy[1],nxy[0]].any() > 0:
            # Good move-to location
            self.xy = nxy

    def getPosition(self):
        return self.xy

