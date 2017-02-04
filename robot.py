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
        self.gearMotor1 = wpilib.Relay(0)
        self.gearMotor2 = wpilib.Relay(1)
        self.gyro = wpilib.ADXRS450_Gyro(0)
        self.accelerometer = wpilib.BuiltInAccelerometer(1)
        self.are = []
        self.counter = 0
        self.camera = 0
                                
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
        if self.stick.getRawButton(4):
            if self.counter < len(self.are):
                self.robot_drive.arcadeDrive(0.75*self.are[self.counter][0],-0.75*self.are[self.counter][1])
                self.counter = self.counter + 1
            else:
                self.robot_drive.arcadeDrive(0,0)
        elif self.getRawButton(2):
                self.robot_drive.arcadeDrive(0,self.camera)
                
        else:
            self.robot_drive.arcadeDrive(0.75*self.stick.getY(),-0.75*self.stick.getX())
     
        

        if self.stick.getRawButton(3):
            self.are.append([self.stick.getY(), self.stick.getX()])

        if self.stick.getRawButton(8):
            self.are = []
            self.counter = 0
        
        if self.stick.getRawButton(5):
            self.climbingMotor.set(1)
        elif self.stick.getRawButton(6):
            self.climbingMotor.set(-1)
        else:
            self.climbingMotor.set(0)


        if self.stick.getRawButton(1) and self.gearSwitch2.get()== False:
            self.gearMotor1.set(wpilib.Relay.Value.kOff)
        elif self.stick.getRawButton(1) and self.gearSwitch2.get():
            self.gearMotor1.set(wpilib.Relay.Value.kReverse)
        elif self.stick.getRawButton(1) == False and self.gearSwitch1.get()== False:
            self.gearMotor1.set(wpilib.Relay.Value.kOff)
        elif self.stick.getRawButton(1) == False and self.gearSwitch1.get():
            self.gearMotor1.set(wpilib.Relay.Value.kForward)
        
    
        if self.stick.getRawButton(1) == False and self.gearSwitch3.get()== False:
            self.gearMotor2.set(wpilib.Relay.Value.kOff)
        elif self.stick.getRawButton(1) == False and self.gearSwitch3.get():
            self.gearMotor2.set(wpilib.Relay.Value.kReverse)
        elif self.stick.getRawButton(1) and self.gearSwitch4.get()== False:
            self.gearMotor2.set(wpilib.Relay.Value.kOff)            
        elif self.stick.getRawButton(1) and self.gearSwitch4.get():
            self.gearMotor2.set(wpilib.Relay.Value.kForward)

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
        self.table.putNumber('GyroAngle', self.gyro.getAngle())
        self.table.putNumber('GyroRate', self.gyro.getRate())
        self.table.putNumber('AccelerometerX', round(self.accelerometer.getX(), 2))
        self.table.putNumber('AccelerometerY', round(self.accelerometer.getY(), 2))
        self.table.putNumber('AccelerometerZ', round(self.accelerometer.getZ(), 2))
        self.table.getNumber('CameraX', 0)
        #self.table.putInt('i', self.counter)
        
    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
