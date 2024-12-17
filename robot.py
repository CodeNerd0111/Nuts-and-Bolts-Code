
import wpilib
import wpilib.drive
import wpimath.controller
import rev
from wpilib.cameraserver import CameraServer
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard
import navx

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        
        # Initializes device channels
        self.XboxController_Channel = Shuffleboard.getTab("Device Ports").add(title="Xbox Controller [USB]", defaultValue=1).getEntry()
        self.l_joystick_Channel = Shuffleboard.getTab("Device Ports").add(title="Left Controller [USB]", defaultValue=0).getEntry()
        self.leftDrive_Channel = Shuffleboard.getTab("Device Ports").add(title="Left Motor [CAN]", defaultValue=21).getEntry()
        self.rightDrive_Channel = Shuffleboard.getTab("Device Ports").add(title="Left Controller [CAN]", defaultValue=22).getEntry()
        self.frontSensor_Channel = Shuffleboard.getTab("Device Ports").add(title="Front Sensor [DIO]", defaultValue=0).getEntry()
        self.backSensor_Channel = Shuffleboard.getTab("Device Ports").add(title="Back Sensor [DIO]", defaultValue=1).getEntry()
        self.gyro_Channel = Shuffleboard.getTab("Device Ports").add(title="Gyro [Analog]", defaultValue=0).getEntry()

        
        
        # Configures CAN Bus Networks to the SparkMax Motor Controllers
        self.leftDrive = rev.CANSparkMax(self.leftDrive_Channel.getInteger(21), rev.CANSparkLowLevel.MotorType.kBrushed)
        self.rightDrive = rev.CANSparkMax(self.rightDrive_Channel.getInteger(22), rev.CANSparkLowLevel.MotorType.kBrushed)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)


        # Sets up the controller channels
        self.controller = wpilib.XboxController(self.XboxController_Channel.getInteger(1))
        self.l_joystick = wpilib.Joystick(self.l_joystick_Channel.getInteger(0))
        self.timer = wpilib.Timer()


        # Links to CameraServer in vision.py
        CameraServer.launch("vision.py:main")


        # Creates two DIO objects to represent the front and back sensors on specified channels
        self.frontSensor = wpilib.Counter(self.frontSensor_Channel.getInteger(0))
        self.backSensor = wpilib.Counter(self.backSensor_Channel.getInteger(1))

        self.frontSensor.setSemiPeriodMode(True)
        self.backSensor.setSemiPeriodMode(True)


        # Add the ultrasonic to the "Sensors" tab of the dashboard
        # Data will update automatically
        Shuffleboard.getTab("Sensors").add(self.frontSensor)
        Shuffleboard.getTab("Sensors").add(self.backSensor)

        # Other data to upload to the dashboard under a new Variable Config tab
        self.clearance = Shuffleboard.getTab("Variable Config").add(title="Sensor Stop Gap", defaultValue=30.0).getEntry()
        self.speed = Shuffleboard.getTab("Variable Config").add(title="Speed Multiplier", defaultValue=1.0).getEntry()
    

        # Other data to upload to the dashboard under a new Variable Config tab
        self.clearance = Shuffleboard.getTab("Variable Config").add(title="Sensor Stop Gap", defaultValue=1.0).getEntry()
        self.speed = Shuffleboard.getTab("Variable Config").add(title="Speed Multiplier", defaultValue=1.0).getEntry()
        self.kVoltsPerDegreePerSecond = Shuffleboard.getTab("Variable Config").add(title="Gyro Sensitivity", defaultValue=0.0128).getEntry()

        # Initilizes gyro object
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB1)



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
        self.returnToStraight = wpimath.controller.PIDController(Kp=1, Ki=0.0, Kd=0.0, period=0.02)
        self.returnToStraight.setSetpoint(0)
        self.returnToStraight.setTolerance(positionTolerance=1, velocityTolerance=1)


    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

        driveVal = -(self.l_joystick.getY()) ** 3
        turnVal = (self.l_joystick.getX()) ** 3

        # Keep the buddy going in a straight line if no turn
        if abs(self.l_joystick.getX()) < 0.1 and abs(self.l_joystick.getY()) < 0.1 and not self.returnToStraight.atSetpoint():
            turnVal = self.returnToStraight.calculate(self.gyro.getYaw())
        SmartDashboard.putNumber("Turn Speed", turnVal)


        # Reads the pulse widths
        # Gets the period in seconds, convert to µs
        self.front_pulse_width = (self.frontSensor.getPeriod()) * 1000000
        self.back_pulse_width = (self.backSensor.getPeriod()) * 1000000

        # Converts the data into cm / No it doesn't :| / Yeab :D
        # 4mm / 1µs ⋅ (t –1000 µs) = distance [mm] ---- https://www.pololu.com/product/4079

        self.front_pulse_cm = (self.front_pulse_width - 1000) * 0.4
        self.back_pulse_cm = (self.back_pulse_width - 1000) * 0.4

        # Publishes the data to the SmartDashboard
        SmartDashboard.putNumber("Front Distance (cm)", self.front_pulse_cm)
        SmartDashboard.putNumber("Back Distance (cm)", self.back_pulse_cm)

        # Publish the data from the gyro
        SmartDashboard.putNumber("Gyro Angle", self.gyro.getYaw())
        SmartDashboard.putNumber("Gyro Rate", self.gyro.getRate())

        
        

        #Keep our buddy away from the walls :)
        maxSpeedForward = ((self.front_pulse_cm / self.clearance.getDouble(1.0)) ** 2) - 1
        maxSpeedBackward = -((self.back_pulse_cm / self.clearance.getDouble(1.0)) ** 2) - 1

        SmartDashboard.putNumber("maxSpeedForward", maxSpeedForward)
        SmartDashboard.putNumber("maxSpeedBackward", maxSpeedBackward)

        driveVal = maxSpeedForward if driveVal > maxSpeedForward else driveVal
        driveVal = maxSpeedBackward if driveVal < maxSpeedBackward else driveVal

        # Controls motor speeds
        # driveVal is the translational motion and turnVal is the rotational motion
        self.robotDrive.arcadeDrive(driveVal, turnVal)






if __name__ == "__main__":
    wpilib.run(MyRobot)
