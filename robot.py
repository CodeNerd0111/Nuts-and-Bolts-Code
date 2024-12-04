
import wpilib
import wpilib.drive
import rev
from wpilib.cameraserver import CameraServer
import wpiutil
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

        """Sets up the controller channels"""
        self.controller = wpilib.XboxController(1)
        self.l_joystick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

        """Links to the Camera"""
        CameraServer.launch("vision.py:main")

        """Links to both sensors"""
        #!!!!!!!!!!!need to change the 1 and 2 on lines 29 and 30 or 39 tests fail (may be false alarm) when they are assigned as such!!!!!!!!!!!!!!!!!!
        self.frontSensor = wpilib.Ultrasonic(0, 1)
        self.backSensor = wpilib.Ultrasonic(2, 3)
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

        self.clearance = 1.0 #in meters
        self.speed = 1.0
        # Creates a ping-response Ultrasonic object on DIO 1 and 2.
        self.rangeFinder = wpilib.Ultrasonic(1, 2)

        # Add the ultrasonic to the "Sensors" tab of the dashboard
        # Data will update automatically
        Shuffleboard.getTab("Sensors").add(self.rangeFinder)
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

        self.robotDrive.arcadeDrive(
            driveVal, turnVal
        )

        # We can read the distance in millimeters
        distanceMillimeters = self.rangeFinder.getRangeMM()
        # ... or in inches
        distanceInches = self.rangeFinder.getRangeInches()

        # We can also publish the data itself periodically
        SmartDashboard.putNumber("Distance[mm]", distanceMillimeters)
        SmartDashboard.putNumber("Distance[in]", distanceInches)

    def testInit(self):
        # By default, the Ultrasonic class polls all ultrasonic sensors every in a round-robin to prevent
        # them from interfering from one another.
        # However, manual polling is also possible -- notes that this disables automatic mode!
        self.rangeFinder.ping()
    def testPeriodic(self):
        if self.rangeFinder.isRangeValid():
            # Data is valid, publish it
            SmartDashboard.putNumber("Distance[mm]", self.rangeFinder.getRangeMM())
            SmartDashboard.putNumber("Distance[in]", self.rangeFinder.getRangeInches())

            # Ping for next measurement
            self.rangeFinder.ping()
    def initSendable(self, builder:wpiutil.SendableBuilder):
        builder.addFloatProperty("Speed", self.speed, self.speed)
        builder.addFloatProperty("Clearance", self.clearance, self.clearance)
    def testExit(self):
        # Enable automatic mode
        self.rangeFinder.setAutomaticMode(True)

if __name__ == "__main__":
    wpilib.run(MyRobot)
