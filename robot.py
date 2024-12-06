
import wpilib
import wpilib.drive
import rev
from wpilib.cameraserver import CameraServer
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Enabled") # Just testing it out seeing if it will output to the command console terminal
        
        # Configures CAN Bus Networks to the SparkMax Motor Controllers
        self.leftDrive = rev.CANSparkMax(21, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.rightDrive = rev.CANSparkMax(22, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

        # Sets up the controller channels
        self.controller = wpilib.XboxController(1)
        self.l_joystick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

        # Links to CameraServer in vision.py
        CameraServer.launch("vision.py:main")

        # Creates a ping-response Ultrasonic object on DIO 1 and 2 and a second on DIO 3 and 4
        self.frontSensor = wpilib.Ultrasonic(1, 2)
        self.backSensor = wpilib.Ultrasonic(3, 4)
        

        # Add the ultrasonic to the "Sensors" tab of the dashboard
        # Data will update automatically
        Shuffleboard.getTab("Sensors").add(self.frontSensor)
        Shuffleboard.getTab("Sensors").add(self.backSensor)

        # Other data to upload to the dashboard under a new Variable Config tab
        self.clearance = Shuffleboard.getTab("Variable Config").add(title="Sensor Stop Gap", defaultValue=1.0).getEntry()
        self.speed = Shuffleboard.getTab("Variable Config").add(title="Speed Multiplier", defaultValue=1.0).getEntry()
        
        # Initilizes gyro object
        self.gyro = wpilib.AnalogGyro(0)
        self.gyro.setSensitivity(self.kVoltsPerDegreePerSecond.getDouble(0.0128))

        # Other data to upload to the dashboard under a new Variable Config tab
        self.clearance = Shuffleboard.getTab("Variable Config").add(title="Sensor Stop Gap", defaultValue=1.0).getEntry()
        self.speed = Shuffleboard.getTab("Variable Config").add(title="Speed Multiplier", defaultValue=1.0).getEntry()
        self.kVoltsPerDegreePerSecond = Shuffleboard.getTab("Variable Config").add(title="Gyro Sensitivity", defaultValue=0.0128).getEntry()

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
        if self.backSensor.getRange() < self.clearance.getDouble(1.0) and driveVal < 0 and self.backSensor.isRangeValid():
            driveVal = 0
        elif self.backSensor.getRange() < self.clearance.getDouble(1.0) and driveVal > 0 and self.frontSensor.isRangeValid():
            driveVal = 0

        # Controls motor speeds
        # driveVal is the translational motion and turnVal is the rotational motion
        self.robotDrive.arcadeDrive(driveVal, turnVal)

        # Publish the data from both sensors
        SmartDashboard.putNumber("F_Distance[mm]", self.frontSensor.getRangeMM())
        SmartDashboard.putNumber("F_Distance[in]", self.frontSensor.getRangeInches())
        SmartDashboard.putNumber("B_Distance[mm]", self.backSensor.getRangeMM())
        SmartDashboard.putNumber("B_Distance[in]", self.backSensor.getRangeInches())

        # Publish the data from the gyro
        SmartDashboard.putNumber("Gyro Angle", self.gyro.getAngle())
        SmartDashboard.putNumber("Gyro Center", self.gyro.getCenter())
        SmartDashboard.putNumber("Gyro Rate", self.gyro.getRate())

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
