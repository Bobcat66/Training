# Glossary

These are terms commonly used in FRC programming

- Accelerometer: A device used to measure acceleration

- AprilTag: A type of fiducial marker commonly used for robot pose estimation in FRC

- Auton: The first 15 seconds of an FRC match, when the robot runs without human input

- Brushed Motor: A brushed motor is a motor that uses an electric brush to deliver power

- Brushless Motor: Brushless motors are lighter and more reliable than brushed motors, and have superseded brushed motors in most applications in FRC

- CAN: Controller Area Network; A protocol commonly used to allow different devices to communicate with each other.

- CIM: A CIM is a big brushed DC motor

- CTRE: Cross The Road Electronics, Inc.; A major supplier of FRC components. Notable CTRE products include the TalonFX and TalonFXS motor controllers, the Pigeon 2 IMU, CANivore, and Minion motor

- Differential Drive: A type of drivetrain in which each side can be controlled independently, similar to a tank. For this reason, differential drive is often referred to as "Tank Drive"

- Multithreading: When multiple processes run in parallel on a computer. In most cases, multithreading on an FRC robot is a bad idea, with a few important exceptions

- IMU: Inertial Measurement Unit; a sensor that combines both a gyroscope and an accelerometer

- I2C: An inter-device communication protocol invented by PCB designers to facilitate communication between integrated circuits. In general, other communications protocols, such as CAN, SPI, and Ethernet are better suited for FRC

- Footgun: A footgun is a language or library feature that can easily cause catastrophic errors if not used correctly, so called because they are the programming equivalent of a gun aimed at your foot.

- RoboRIO: A RoboRIO is the main control system of a robot. Your robot code runs on the RoboRIO

- Pose: A robot's "Pose" is its position on the field

- Pose Estimation: Pose Estimation is the task of determining a robot's position on the field. Being able to do it well is essential for pretty much any sort of automation

- Th PnP Problem: The PnP Problem is an extremely important problem in computer vision, which, massively oversimplified, essentially asks the question "If you have a picture of an object with a known location, can you deduce the location of the camera that took it?"

- Factor Graph: A bipartite graph representing the factorization of a probability distribution function. They are commonly utilized in computer vision and robotics to break up complex problems into smaller parts that are easier to solve.

- Kalman Filter: A more computationally efficient (albeit less accurate) pose estimation algorithm used by WPILib for on-RIO pose estimation, at least when compared to Factor-Graph based solutions

- Control Systems: A control system is an algorithm used to control a mechanism on the robot

- Plant: The system being controlled by a control system

- Setpoint: The desired state of a plant

- Process Variable: The actual state of a plant

- Error: The difference between the setpoint and the process variable

- PID: PID, or Proportional-Integral-Derivative, is by far the most common control system utilized in FRC. A PID controller can be fully described by three coefficients: the proportional coefficient $K_p$, the integral coefficient $K_i$, and the derivative coefficient $K_d$. A PID controller's output at time t is described by the following equation: 
$$K_pe(t) + K_i\int e(t)dt + K_d\frac{de}{dt}$$




