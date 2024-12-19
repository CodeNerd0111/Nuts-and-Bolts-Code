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
D_Kp = 1.3
D_Ki = 0.1
D_Kd = 0.5
D_posTolerance = 2
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
   .addPersistent("Kp", D_Kp)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
Ki = (PIDtab                                     #Ki
   .addPersistent("Ki", D_Ki)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
Kd = (PIDtab                                     # Kd
   .addPersistent("Kd", D_Kd)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())
posTolerance = (PIDtab                         # Tolerance
   .addPersistent("Tolerance", D_posTolerance)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .getEntry())