#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.IterativeRobot):
    kP = 0.01
    kI = 0.02
    kD = 0.03
    kF = 0.00
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
        self.gearMotor1 = wpilib.Victor(7)
        self.gearMotor2 = wpilib.Victor(8)
        self.ballMotor1 = wpilib.Relay(2)
        self.gyro = wpilib.ADXRS450_Gyro(0)
        self.accelerometer = wpilib.BuiltInAccelerometer(1)
        self.are = []
        self.counter = 0
        self.camera = 0
        self.gearSpeed = .5 
        self.gearSpeedUp = True
        self.gearSpeedDown = True
        self.rotateToAngleRate = 0

        turnController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.gyro.getAngle, output=self)
        turnController.setInputRange(-180.0,  180.0)
        turnController.setOutputRange(-1.0, 1.0)
        turnController.setAbsoluteTolerance(5)
        turnController.setContinuous(True)
        
        self.turnController = turnController
                                
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
        rotateToAngle = False

        if self.stick.getRawButton(2):
            self.turnController.setSetpoint(0.0)
            rotateToAngle = True
        elif self.stick.getRawButton(3):
            self.turnController.setSetpoint(90.0)
            rotateToAngle = True
        elif self.stick.getRawButton(4):
            self.turnController.setSetpoint(179.9)
            rotateToAngle = True
        elif self.stick.getRawButton(5):
            self.turnController.setSetpoint(-90.0)
            rotateToAngle = True
            
        if rotateToAngle:
            self.turnController.enable()
            currentRotationRate = self.rotateToAngleRate
        else:
            self.turnController.disable()
            currentRotationRate = -0.75*self.stick.getX()

        self.robot_drive.arcadeDrive(0.75*self.stick.getY(),currentRotationRate)
 

        self.table.putNumber('rot', self.rotateToAngleRate)
        self.table.putNumber('cur', currentRotationRate)
        self.table.putNumber('GyroRate', self.gyro.getRate())
        self.table.putNumber('AccelerometerX', round(self.accelerometer.getX(), 2))
        self.table.putNumber('AccelerometerY', round(self.accelerometer.getY(), 2))
        self.table.putNumber('AccelerometerZ', round(self.accelerometer.getZ(), 2))
        self.table.getNumber('CameraX', 0)
        self.table.putBoolean('ballSwitch1', self.ballSwitch1.get())
        self.table.putBoolean('ballSwitch2', self.ballSwitch2.get())
        #self.table.putInt('i', self.counter)

    def pidWrite(self, output):
        """This function is invoked periodically by the PID Controller,
        based upon navX MXP yaw angle input and PID Coefficients.
        """
        self.rotateToAngleRate = output

    def pidOutput(self):
        return self.gyro.getAngle()
    
    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
