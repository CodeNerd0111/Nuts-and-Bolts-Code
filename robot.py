
import wpilib
import wpilib.drive
import rev
from wpilib.cameraserver import CameraServer
import wpiutil

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
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
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

        self.speed = 1.0

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
        self.robotDrive.arcadeDrive(
            self.l_joystick.getX() * self.speed, self.l_joystick.getZ() * self.speed
        )

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""

    def initSendable(self, builder:wpiutil.SendableBuilder):
        builder.addFloatProperty("Speed", self.speed, self.speed)

if __name__ == "__main__":
    wpilib.run(MyRobot)
