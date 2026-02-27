# Helper class for inverse and forward kinematics, motion planning

import math as m
import numpy as np
import json

from enum import Enum

from itertools import product

from colors import COLOR_RGB, pickRandomColor

np.set_printoptions(suppress=True, precision=2)


class ARM_5DOF:


    def __init__(self):
        
        linkLengths = [100, 100, 100, 100, 100]
        initialAngles = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.homeAngles = initialAngles.copy()

        self.jointAngles = initialAngles.copy()
        self.jointCoordinates = [[0.0, 0.0, 0.0], [], [], [], [], []] # default mm
        self.jointCoordinatesM = [[0.0, 0.0, 0.0], [], [], [], [], []] # non default m

        self.dhParams = {}
        self.dhParams["alpha"] = [m.pi/2, 0.0, 0.0, m.pi/2, 0.0]
        self.dhParams["d"] = [linkLengths[0], 0.0, 0.0, 0.0, linkLengths[3] + linkLengths[4]]
        self.dhParams["a"] = [0.0, linkLengths[1], linkLengths[2], 0.0, 0.0]

        self.status = "ready"


    def stateToDict(self):
        return {
            "type": "robot_state",
            
            "home_angles_rad": self.homeAngles,

            "joint_angles_rad": self.jointAngles,

            "joint_coordinates_m": self.jointCoordinatesM,

            "status": self.status,
        }
    

    def toJSON(self):
        return json.dumps(self.stateToDict, indent=4)

    def printArrayPretty(self, array):
        print(' '.join(f"{val:6.2f}" for val in array))
        print("")
    
    
    def printMatrixPretty(self, matrix):
        for row in matrix:
            print(' '.join(f"{val:6.2f}" for val in row))
        print("")


    def extractCoordinates(self, tf_mat):
        # extract x, y, z position of a particular joint given its T0i tf matrix      
        return [tf_mat[0][3], tf_mat[1][3], tf_mat[2][3]]
    

    def tfMatrix(self, i):

        # extract variables from dict for verbosity
        theta = self.dhParams["theta"][i]
        alpha = self.dhParams["alpha"][i]
        a = self.dhParams["a"][i]
        d = self.dhParams["d"][i]

        # from standard layout of denavit-hartenburg matrix
        return np.array([[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)], 
                         [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(theta)], 
                         [0.0, m.sin(alpha), m.cos(alpha), d], 
                         [0.0, 0.0, 0.0, 1.0]])


    def solveFK(self, jointAngles):

        # set 'theta' field of dh matrix
        self.dhParams["theta"] = jointAngles
        jointCoordinates = [[0.0, 0.0, 0.0], [], [], [], [], []] # default mm
        
        # generate tf matrices
        product = self.tfMatrix(0)
        jointCoordinates[1] = self.extractCoordinates(product)
        for i in range(1, 5):
            product = np.matmul(product, self.tfMatrix(i))
            jointCoordinates[i+1] = self.extractCoordinates(product) 
        
        #self.printMatrixPretty(self.jointCoordinatesM)
        return jointCoordinates
        
    
    # geometric 5DOF inverse kinematics solver
    # assumes that the target pose is in the form of [x, y, z, theta4, theta5]
    # where theta4 is given WRT world horizonal frame (xy plane)
    # example: wrist straight down would be theta4 = -90 degrees
    def solveIK(self, targetPose):
        # unpack inputs for clarity
        x, y, z, theta4, theta5 = targetPose
        l1, l2, l3, l4, l5 = self.linkLengths

        # convert to radians
        theta4 = m.radians(theta4)
        theta5 = m.radians(theta5)

        # base yaw
        theta1 = m.atan2(y, x)

        # effective wrist offset
        wrist_len = l4 + l5
        horz = wrist_len * m.cos(theta4)
        x4 = x - horz * m.cos(theta1)
        y4 = y - horz * m.sin(theta1)
        z4 = z - wrist_len * m.sin(theta4) - l1

        # distance from shoulder to wrist
        rsquared = x4**2 + y4**2 + z4**2

        # elbow
        cos_phi = (rsquared - l2**2 - l3**2) / (2.0 * l2 * l3)
        theta3 = m.acos(cos_phi)

        # shoulder
        alpha = m.asin(z4 / m.sqrt(rsquared))
        beta = m.atan(l3 * m.sin(theta3) / (l2 + l3 * m.cos(theta3)))
        theta2 = alpha + beta 

        # flip elbow angle to match arm orientation
        # AFTER it is used to calculate theta2
        theta3 *= -1 

        # apply offsets to theta4 as it is provided in world frame
        # NOT last link frame, and FK expects last link frame
        j4_offset = theta2 + theta3 
        theta4 -= j4_offset - m.pi/2

        return [theta1, theta2, theta3, theta4, theta5]
    




    
