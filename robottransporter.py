#!/usr/bin/env python3
import wpilib
from networktables import NetworkTables



class MyRobot(wpilib.IterativeRobot):
    kP = 0.03
    kI = 0.00
    kD = 0.00
    kF = 0.00
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.table = NetworkTables.getTable("SmartDashboard")
        self.robot_drive = wpilib.RobotDrive(0,1)
        self.stick = wpilib.Joystick(0)
        self.transporterSpeed = .2
        self.transporterSpeedUp = True
        self.transporterSpeedDown = True

        turnController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.gyro, output=self)
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
            currentRotationRate = -0.2*self.stick.getX()

        self.robot_drive.arcadeDrive(0.2*self.stick.getY(),currentRotationRate)
        

        if self.stick.getRawButton(2) == False and self.transporterSpeedDown:
            pass
        elif self.stick.getRawButton(2) == False and self.transporterSpeedDown == False:
            self.transporterSpeedDown = True
        elif self.stick.getRawButton(2) and self.transporterSpeedDown:
            self.transporterSpeed = self.transporterSpeed -.05
            self.transporterSpeedDown = False
        elif self.stick.getRawButton(2) and self.transporterSpeedDown == False:
            pass

        if self.stick.getRawButton(1) == False and self.transporterSpeedUp:
            pass
        elif self.stick.getRawButton(1) == False and self.transporterSpeedUp == False:
            self.transporterSpeedUp = True
        elif self.stick.getRawButton(1) and self.transporterSpeedUp:
            self.transporterSpeed = self.transporterSpeed +.05
            self.transporterSpeedUp = False
        elif self.stick.getRawButton(1) and self.transporterSpeedUp == False:
            pass

        self.table.putNumber('stickX', self.stick.getX())
        self.table.putNumber('stickY', self.stick.getY())

        #self.table.putInt('i', self.counter)

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
