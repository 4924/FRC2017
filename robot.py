#!/usr/bin/env python3
import wpilib

class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robot_drive = wpilib.RobotDrive(0,1)
        self.stick = wpilib.Joystick(0)
        self.rawButton = True
        self.robotSpeed = False

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Check if we've completed 100 loops (approximately 2 seconds)
        if self.auto_loop_counter < 100:
            self.robot_drive.drive(-0.5, 0) # Drive forwards at half speed
            self.auto_loop_counter += 1
        else:
            self.robot_drive.drive(0, 0)    #Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        
        if(self.stick.getRawButton(0) and self.rawButton == True):
            #clicked and value is true
            self.robotSpeed = !self.robotSpeed
            self.rawButton = False
        elif(self.stick.getRawButton(0) and self.rawButton == False):
            #clicked and value is false        
        elif(!self.stick.getRawButton(0) and self.rawButton == False):
            #not clicked and value is true
            self.rawButton = True
        
        if(self.robotSpeed):
            self.robot_drive.arcadeDrive(self.stick.getY(),self.stick.getX()) 
        else: 
            self.robot_drive.arcadeDrive(self.stick.getY()/2,self.stick.getX()/2)

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
