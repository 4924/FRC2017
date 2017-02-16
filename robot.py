#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables



class MyRobot(wpilib.IterativeRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.table = NetworkTables.getTable("SmartDashboard")
        self.robot_drive = wpilib.RobotDrive(0,1)
        self.stick = wpilib.Joystick(0)
        self.climbingMotor = wpilib.Talon(2)
        self.gearSwitch1 = wpilib.DigitalInput(0)
        self.gearSwitch2 = wpilib.DigitalInput(1)
        self.gearSwitch3 = wpilib.DigitalInput(2)
        self.gearSwitch4 = wpilib.DigitalInput(3)
        self.ballSwitch1 = wpilib.DigitalInput(4)
        self.ballSwitch2 = wpilib.DigitalInput(5)
        self.gearMotor1 = wpilib.Spark(4)
        self.gearMotor2 = wpilib.Spark(3)
        self.ballMotor1 = wpilib.Relay(0)
        self.gearSpeed = .5 
        self.gearSpeedUp = True
        self.gearSpeedDown = True
        self.lights = wpilib.Relay(1)
        self.lightToggle = False
        self.lightToggleBool = True

                                
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Check if we've completed 100 loops (approximately 2 seconds)
        if self.auto_loop_counter < 100:
            self.robot_drive.drive(-0.5, 0) # Drive forwards at half speed
            self.auto_loop_counter += 1;
        else:
            self.robot_drive.drive(0, 0)    #Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.robot_drive.arcadeDrive(0.75*self.stick.getY(),-0.75*self.stick.getX())
        
     
        
        if self.stick.getRawButton(3):
            self.climbingMotor.set(-.3)
        else:
            self.climbingMotor.set(0)

        if self.stick.getRawButton(4) and self.lightToggleBool == False:
            pass
        elif self.stick.getRawButton(4) and self.lightToggleBool:
            self.lightToggle = not self.lightToggle
            if self.lightToggle:
                self.lights.set(wpilib.Relay.Value.kForward)
            else:
                self.lights.set(wpilib.Relay.Value.kOff)
            self.lightToggleBool = False
        elif self.stick.getRawButton(4) == False and self.lightToggleBool == False:
            self.lightToggleBool = True
    


        if self.stick.getRawButton(1) and self.gearSwitch2.get()== False:
            self.gearMotor1.set(0)
        elif self.stick.getRawButton(1) and self.gearSwitch2.get():
            self.gearMotor1.set(self.gearSpeed)
        elif self.stick.getRawButton(1) == False and self.gearSwitch1.get()== False:
            self.gearMotor1.set(0)
        elif self.stick.getRawButton(1) == False and self.gearSwitch1.get():
            self.gearMotor1.set(-self.gearSpeed)
        
    
        if self.stick.getRawButton(1) == False and self.gearSwitch3.get()== False:
            self.gearMotor2.set(0)
        elif self.stick.getRawButton(1) == False and self.gearSwitch3.get():
            self.gearMotor2.set(self.gearSpeed)
        elif self.stick.getRawButton(1) and self.gearSwitch4.get()== False:
            self.gearMotor2.set(0)            
        elif self.stick.getRawButton(1) and self.gearSwitch4.get():
            self.gearMotor2.set(-self.gearSpeed)

        if self.stick.getRawButton(2) == False and self.ballSwitch1.get()==False:
            self.ballMotor1.set(wpilib.Relay.Value.kOff)
        elif self.stick.getRawButton(2) == False and self.ballSwitch1.get():
            self.ballMotor1.set(wpilib.Relay.Value.kReverse)
        elif self.stick.getRawButton(2) and self.ballSwitch2.get()==False:
            self.ballMotor1.set(wpilib.Relay.Value.kOff)            
        elif self.stick.getRawButton(2) and self.ballSwitch2.get():
            self.ballMotor1.set(wpilib.Relay.Value.kForward)

        if self.stick.getRawButton(9) == False and self.gearSpeedDown:
            pass
        elif self.stick.getRawButton(9) == False and self.gearSpeedDown == False:
            self.gearSpeedDown = True
        elif self.stick.getRawButton(9) and self.gearSpeedDown:
            self.gearSpeed = self.gearSpeed -.05
            self.gearSpeedDown = False
        elif self.stick.getRawButton(9) and self.gearSpeedDown == False:
            pass

        if self.stick.getRawButton(10) == False and self.gearSpeedUp:
            pass
        elif self.stick.getRawButton(10) == False and self.gearSpeedUp == False:
            self.gearSpeedUp = True
        elif self.stick.getRawButton(10) and self.gearSpeedUp:
            self.gearSpeed = self.gearSpeed +.05
            self.gearSpeedUp = False
        elif self.stick.getRawButton(10) and self.gearSpeedUp == False:
            pass

        self.table.putNumber('stickX', self.stick.getX())
        self.table.putNumber('stickY', self.stick.getY())
        self.table.putNumber('Switch1', self.gearSwitch1.get())
        self.table.putNumber('Switch2', self.gearSwitch2.get())
        self.table.putNumber('Switch3', self.gearSwitch3.get())
        self.table.putNumber('Switch4', self.gearSwitch4.get())
        self.table.putNumber('GearMotor1 Forward', self.gearMotor1.get())
        self.table.putNumber('GearMotor2 Forward', self.gearMotor2.get())
        self.table.putNumber('GearMotor1 Reverse', self.gearMotor1.get())
        self.table.putNumber('GearMotor2 Reverse', self.gearMotor2.get())

    def pidWrite(self, output):
        """This function is invoked periodically by the PID Controller,
        based upon navX MXP yaw angle input and PID Coefficients.
        """
        self.rotateToAngleRate = output
        
    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
