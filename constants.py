from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets

# Device Ports
p_leftDrive = 21
p_rightDrive = 22
p_Xbox = 1
p_joystick = 0
p_frontSensor = 0
p_backSensor = 1

# Default Gyro PID Constants
D_Kp = 4.59
D_Ki = 2.87
D_Kd = 1.15
D_posTolerance = .15
velTolerance = 1
setPoint = 0
period = 0.02

# Default Variables
speed = 1
clearance = 30
joyDead = 0.1

# Shuffleboard widgets for PID coefficients
PIDtab = Shuffleboard.getTab("PID")
Kp = (PIDtab                                     # Kp
   .add("Kp", D_Kp)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
Ki = (PIDtab                                     #Ki
   .add("Ki", D_Ki)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
Kd = (PIDtab                                     # Kd
   .add("Kd", D_Kd)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
posTolerance = (PIDtab                         # Tolerance
   .add("Tolerance", D_posTolerance)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())