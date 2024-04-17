import obi
import time
from datetime import date

class ObiJointNode():
    def __init__(self):
        #Note – don't think there's a good analog for this in Max OSX
        #will probably need to connect a lab machine to the robot for ease of use
        self.robot = obi.Obi('/dev/ttyUSB0')
        print (self.robot.SerialIsOpen())
        print (self.robot.VersionInfo())

    def collect_joint_positions(self):
        #Collects joint positions specified by user (w/ custom names) and writes to file
        myuser_input = input('Collect new joint position? (y/n): ')
        if (myuser_input == "y"):
            name = input('Enter name for joint position set: ')
            new_pos = self.robot.MotorPositions()
            print(name + ": " + str(new_pos))
            new_pos = [name]+[str(x) for x in new_pos]

            #Write saved joint positions to file
            file = open("saved_joint_positions/" + str(date.today()) + '.txt', "a")
            file.write('\n')
            file.write((' '.join(new_pos)))
            file.close()

            time.sleep(2)

    def execute_joint_positions(self,stored_joint_positions):
        #Executes list of joint positions specified by user
        print(stored_joint_positions)

        joint_position = stored_joint_positions.split(' ')
        print(joint_position)
        for i in range(1,7):
            print(i-1, int(joint_position[i]))
            self.robot.MoveMotorToAbsolutePosition(i-1,joint_position[i])
            time.sleep(7)
        time.sleep(20)

    def collect_positions_while_moving(self):
        file = open("saved_joint_positions/" + str(date.today()) + '_waypoints.txt', "a")
        for i in range (0,10):
            new_pos = self.robot.MotorPositions()
            new_pos = [str(x) for x in new_pos]
            print("collected " + str(new_pos))
            file.write((' '.join(new_pos)))
            file.write('\n')
            time.sleep(.5)
        file.close()

    

if __name__ == '__main__':
    user_input = input('Wake? (y/n): ')
    obinode = ObiJointNode()

    if (user_input == "y"):
        obinode.robot.Wakeup()
        time.sleep(20)

    print("Current position: ")
    print(obinode.robot.MotorPositions())
    obinode.collect_joint_positions()

    new_user_input = input('Execute new position? (y/n): ')  

    while (new_user_input == "y"):
        desired_posn = input('Desired Position Set? ')
        file = open("saved_joint_positions/" + str(date.today()) + '.txt', "r")
        posns = (file.read()).split('\n')
        posn = None
        for p in posns:
            if (p.split(' '))[0] == desired_posn:
                posn = p
                break
        if posn == None:
            obinode.robot.GoToSleep()
        obinode.execute_joint_positions(posn)
        new_user_input = input('Execute new position? (y/n): ')  

    obinode.robot.GoToSleep()
    obinode.robot.Close()

