
import wpilib
import wpilib.drive
import rev
from wpilib.cameraserver import CameraServer
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Enabled") #just testing it out seeing if it will output to the command consol terminal
        """Configures CAN Bus Networks to the SparkMax Motor Controllers"""
        self.leftDrive = rev.CANSparkMax(21, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.rightDrive = rev.CANSparkMax(22, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

        """Sets up the controller channels"""
        self.controller = wpilib.XboxController(1)
        self.l_joystick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

        """Links to the Camera"""
        CameraServer.launch("vision.py:main")

        """Links to both sensors"""
        # Creates a ping-response Ultrasonic object on DIO 1 and 2 and a second on DIO 3 and 4
        #!!!!!!!!!!!need to change the 1 and 2 on lines 29 and 30 or 39 tests fail (may be false alarm) when they are assigned as such!!!!!!!!!!!!!!!!!! --- Resolved(?)
        self.frontSensor = wpilib.Ultrasonic(1, 2)
        self.backSensor = wpilib.Ultrasonic(3, 4)

        self.clearance = 1.0 #in meters
        self.speed = 1.0
        

        # Add the ultrasonic to the "Sensors" tab of the dashboard
        # Data will update automatically
        Shuffleboard.getTab("Sensors").add(self.frontSensor)
        Shuffleboard.getTab("Sensors").add(self.backSensor)
        # Other data to upload to the dashboard under a new Variable Config tab
        Shuffleboard.getTab("Variable Config").addNumber("Sensor Stop Gap", self.clearance)
        Shuffleboard.getTab("Variable Config").addNumber("Speed Multiplier", self.speed)
        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            # Drive forwards half speed, make sure to turn input squaring off
            self.robotDrive.arcadeDrive(0.5, 0, squareInputs=False)
        else:
            self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""


    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        driveVal = -self.l_joystick.getY()
        turnVal = self.l_joystick.getX()
        if self.backSensor.getRange() < self.clearance and driveVal < 0 and self.backSensor.isRangeValid():
            driveVal = 0
        elif self.backSensor.getRange() < self.clearance and driveVal > 0 and self.frontSensor.isRangeValid():
            driveVal = 0

        # Controls moter speeds
        # driveVal is the translational motion and turnVal is the rotational motion
        self.robotDrive.arcadeDrive(driveVal, turnVal)

        # Publish the data periodically
        SmartDashboard.putNumber("F_Distance[mm]", self.frontSensor.distanceMillimeters)
        SmartDashboard.putNumber("F_Distance[in]", self.frontSensor.distanceInches)
        SmartDashboard.putNumber("B_Distance[mm]", self.backSensor.getRangeMM())
        SmartDashboard.putNumber("B_Distance[in]", self.backSensor.getRangeInches())

    def testInit(self):
        # By default, the Ultrasonic class polls all ultrasonic sensors every in a round-robin to prevent
        # them from interfering from one another.
        # However, manual polling is also possible -- notes that this disables automatic mode!
        self.frontSensor.ping()
    def testPeriodic(self):
        if self.frontSensor.isRangeValid():
            # Data is valid, publish it
            SmartDashboard.putNumber("F_Distance[mm]", self.frontSensor.getRangeMM())
            SmartDashboard.putNumber("F_Distance[in]", self.frontSensor.getRangeInches())

            # Ping for next measurement
            self.frontSensor.ping()
    def testExit(self):
        # Enable automatic mode
        self.frontSensor.setAutomaticMode(True)

if __name__ == "__main__":
    wpilib.run(MyRobot)
