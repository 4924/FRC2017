#!/usr/bin/env python3
import wpilib
from networktables import NetworkTable
from networktables.util import ntproperty

class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        table = NetworkTables.getTable('dash')
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

        table.putNumber('stickX', self.stick.getX())
        table.putNumber('stickY', self.stick.getY())
        table.putNumber('Switch1', self.gearSwitch1.get())
        table.putNumber('Switch2', self.gearSwitch2.get())
        table.putNumber('Switch3', self.gearSwitch3.get())
        table.putNumber('Switch4', self.gearSwitch4.get())
        table.putNumber('GearMotor1 Forward', self.gearMotor1.set(wpilib.Relay.Value.kForward))
        table.putNumber('GearMotor2 Forward', self.gearMotor2.set(wpilib.Relay.Value.kForward))
        table.putNumber('GearMotor1 Reverse', self.gearMotor1.set(wpilib.Relay.Value.kReverse))
        table.putNumber('GearMotor2 Reverse', self.gearMotor2.set(wpilib.Relay.Value.kReverse))
        
    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
