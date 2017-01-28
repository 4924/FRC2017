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
        self.climbingMotor = wpilib.Talon(2)
        self.gearSwitch1 = wpilib.DigitalInput(1)
        self.gearSwitch2 = wpilib.DigitalInput(2)
        self.gearSwitch3 = wpilib.DigitalInput(3)
        self.gearSwitch4 = wpilib.DigitalInput(4)
        self.gearMotor1 = wpilib.Relay(1)
        self.gearMotor2 = wpilib.Relay(2)
                                
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
        
        self.robot_drive.arcadeDrive(self.stick.getY()/2,self.stick.getX()/2)
        if self.stick.getRawButton(5):
            self.climbingMotor.set(1)
        elif self.stick.getRawButton(6):
            self.climbingMotor.set(-1)
        else:
            self.climbingMotor.set(0)

        if not self.stick.getRawButton(1) and self.gearSwitch1.get():
            self.gearMotor1.set(wpilib.Relay.Value.kOff)
        elif not self.stick.getRawButton(1) and self.gearSwitch1.get():
            self.gearMotor1.set(wpilib.Relay.Value.kForward)
        elif self.stick.getRawButton(1) and self.gearSwitch2.get():
            self.gearMotor1.set(wpilib.Relay.Value.kOff)
        elif self.stick.getRawButton(1) and self.gearSwitch2.get():
            self.gearMotor1.set(wpilib.Relay.Value.kReverse)
        
    
        if not self.stick.getRawButton(1) and self.gearSwitch3.get():
            self.gearMotor2.wpilib.Relay.Value.kOff)
        elif not self.stick.getRawButton(1) and self.gearSwitch3.get():
            self.gearMotor2.set(wpilib.Relay.Value.kReverse)
        elif self.stick.getRawButton(1) and self.gearSwitch4.get():
            self.gearMotor2.set(wpilib.Relay.Value.kOff)            
        elif self.stick.getRawButton(1) and self.gearSwitch4.get():
            self.gearMotor2.set(wpilib.Relay.Value.kForward)
        
    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
