import math as m

from .kinematic_helper import ARM_INFO

class UserInputHandler:


    def __init__(self, robot):
        
        self.robot = robot
        
        self.command_list = ["help - Show this help message", 
                "home - Move robot to home position",
                "print - print robot end effector position",
                "ik - Move to an x,y,z,r,p position in space",
                "fk - Enter a joint angle in radians"]
        

    def generateMovement(self, goal_pose):

        x, y, z, r, p = goal_pose

        if self.robot.movement_type == ARM_INFO.JOINT_SPACE:
            self.robot.joint_angles = self.robot.solveIK([x, y, z, r, p])
            self.printListFormatted(self.robot.joint_angles)
            test_fk = self.robot.solveFK(self.robot.joint_angles)

        elif self.robot.movement_type == ARM_INFO.PATH_SPACE:
            update_delay = 50 #ms
            n = int(self.robot.path_speed/1000.0 / self.robot.path_resolution * update_delay) # number of points to skip for animation
            self.robot.path_points, self.robot.path_angles = self.robot.linePath([x, y, z, r, p])
            self.robot.sliced_path_points, self.robot.sliced_path_angles = self.robot.slicePath(self.robot.path_points, self.robot.path_angles, n)
            print(f'Generated path with {len(self.robot.sliced_path_points)} points.')

    def parseUserInput(self, user_input):

        list = user_input.split()

        if list[0] == "help":
            for command in self.command_list:
                print(command)

        elif list[0] == "home":
            self.robot.joint_angles = self.robot.home_angles.copy()

        elif list[0] == "print":
            self.printListFormatted(self.robot.joint_coordinates[5])

        elif list[0] == "fk":
            if len(list) != 3:
                print("Invalid command. Usage: fk <joint> <angle>")
                return
            joint = int(list[1]) 
            new_angle = m.radians(float(list[2]))
            self.robot.joint_angles[joint] = new_angle

        elif list[0] == "ik":
            
            self.robot.movement_type = ARM_INFO.JOINT_SPACE
            
            if len(list) != 6:
                print("Invalid command. Usage: ik <x> <y> <z> <r> <p>")
                return
            
            list = [float(x) for x in list[1:]]
            self.generateMovement(list)    

        elif list[0] == "line":

            self.robot.movement_type = ARM_INFO.PATH_SPACE

            if len(list) != 6:
                print("Invalid command. Usage: line <x> <y> <z> <r> <p>")
                return
            
            list = [float(x) for x in list[1:]]
            self.generateMovement(list)  
            
            




    
    def printListFormatted(self, list):
        formatted = [f"{x:.3f}" for x in list]
        print(formatted)

        

    
    