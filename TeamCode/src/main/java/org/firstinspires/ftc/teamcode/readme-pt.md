# Programs #
This is a list of our three main programam, their usage and methods

## MMMovement ##
### My Mechanum: Movement ###
It contains all the methods used to move the robot during the TeleOp and also in autonomous period.
The methods are:

**Methods used to define the hardware's configuration or some abstract actions**
- **defHardware:** It receive the hardwareMap to map the robot's components in our program and in
the robot configuration.
``` java
public void defHardware (HardwareMap hardwareMap)
```
- **initIMU:** It receive hardwareMap and create the IMU configuration.
``` java
public void initIMU (HardwareMap hardwareMap)
```
- **resetEncoder:** It is self-explanatory
``` java
public void resetEncoder()
```

**Methods used to move the robot**
- **turn:** It receive the motors' force, the direction (left or right), the angle and the
 [smoother](#robot-mapping) value.
``` java
public void turn(double force, boolean isRight, double targetAngle, final double smoother)
```
- **move:** It receive the gamepad controls to move the robot using the IMU
``` java
public void move(double leftY, double leftX, double rightX, boolean slower, boolean faster)
```
- **setPd:** It receive the gamepad control and use this to define the angle that the robot should
 be facing
``` java
public void setPd(double rightX)
```
- **intakeForce:** It receive the target force to the intake
``` java
public void intakeForce(double force)
```
- **shoot:** It receive the gamepad key used to shot an ring into the High Goal
``` java
public void shoot(boolean trigger)
```
- **powerShot:** It receive the gamepad key used to shot an ring into the Power Shot
``` java
public void powerShot(boolean trigger)
```
- **claw:** It receive the gamepad keys used to move the claw
``` java
public void claw(boolean halfDown, boolean totalUp, boolean totalDown, boolean open, boolean close)
```

## MMCore ##
### My Mechanum: Core ###
The main TeleOp program, it contains the OpMode and the gamepad controls that we will use.

First it will define the Hardware and initialize the IMU, after this it will start a loop that will
 use the methods created in MMMovement and the gamepad to move the robot

This class has no methods

## MATflowCore ##
### My Autonomous: Tensor Flow Core ###

The main autonomous program, it contains an linear OpMode and some methods used to drive the robot.
 The methods are: 

- **goWhite:** it moves the robot forward until it detects a white tape in the floor
``` java
private void goWhite()
```
- **goZoneA:** it contais the instruction to move the robot when there are no rings in front of it 
``` java
private void goZoneA()
```
- **goZoneB:** it contais the instruction to move the robot when there are one rings in front of it
``` java
private void goZoneB()
```
- **goZoneC:** it contais the instruction to move the robot when there are four rings in front of it
``` java
private void goZoneC()
```
- **initVuforia:** Initialize the vuforia and the webcam
``` java
private void initVuforia()
```
- **initTfod:** Initialize the TensorFlow, library that detect the objects using vuforia 
``` java
private void initTfod()
```

## Smoother ##

Different than everything else, this is a part of the program that appears in many movement methods,
 it is responsible to make our robot slow down after an movement, making it more precise.
 It is extremely useful and cam be implemented in two ways:
- **Simpler:**
To explain the way it works, i will use the example present in our *turn* method.

The smoother value is with the subtraction between the target angle of our turn with the current
robot's angle. The result will be divided with the Smoother constant that, in most cases, will be
defined inside the method.

``` java
(angle - currentAngle) / smoother
```

With the smoother value on, we can use it with the maximum desired force, let's pretend that it's
0.5, we can use an if-else to define the motors force. If the desired force is smaller, we use it,
if isn't, we use the smoother calculation.

``` java 
SmootherCalc = (angle - currentAngle) / smoother; 

if (SmootherCalc > desiredForce) {
    movementMotor.setPower(desiredForce)
} else{
    movementMotor.setPower(SmootherCalc)
}
``` 

- **Harder:**
This way is only used in the TeleOp movement method and is harder to implement, but I will try to
simplify it in here

To implement this, we will need to have:
1.  The current motor force
1.  The last updated motor force
1.  The smoother constant

With this, we will compare if the absolute value of the difference between the current and last
force is bigger than the smoother constant

``` java
Math.abs( lastForce - currentForce ) > smoother
``` 

if it isn't we can directly define the force to the motors, but if isn't, we will need to change the
value with some logic

if the last force was bigger than the current force, we will use it less the constant smoother as
the motor force, else, we will use the last force plus the smoother as the force

``` java
if (Math.abs(lastForce - currentForce) > smoother) {
    if (lastForce > currentForce) {
        movementMotor.setPower(lastForce - smoother)
    } else {
        movementMotor.setPower(lastForce + smoother)
    }
} else {
    movementMotor.setPower(currentForce)
}
``` 

## PID ##
PID is an abbreviation
 
 
 
# Robot mapping #
Robot Mapping help us with cable management, it is used to avoid unnecessary changes in the cables
 and the complications that come with it.

We had put some colorful tags in our robot's cables to help us identifying it in an easier way.
## HUB 1 / Blue ##
### Motors ###
| Port | Name          | Color  |
|------|---------------|--------|
|  0   | arm_motor     | Yellow |
|  1   | intake_motor  | Blue   |
|  2   | shooter_motor | White  |

### Servos ###
| Port | Name          |
|------|---------------|
|  0   | claw_servo    |
|  1   | trig_servo    |

### I2C 0 ###
| Port | Name          |
|------|---------------|
|  0   | imu           |
|  1   | sensor_color  | 

## HUB 2 / Pink tag ##
### Motors ###
| Port | Name          | Color   |
|------|---------------|---------|
|  0   | FL            | Yellow  |
|  1   | FR            | Blue    |
|  2   | BL            | White   |
|  3   | BR            | Green   |

# Gamepad #
We configured only one gamepad, we had some brainstorms and decided that only one pilot would be
better than two.

the gamepad mapping is:

| Button | Function          |
|--------|-------------------|
| X      | Open the claw     |
| Y      | Lift up the arm   |
| A      | Lift down the arm |
| B      | Close the claw    |
| Right bumper   | Speed up the robot |
| Right trigger  | Shoot the rings (Power Shot) |
| Left trigger   | Shoot the rings (High Goal)  |
| DPad up        | Start/stop the intake        |
| DPad down      | Lift down the arm half way   |
| Right joystick | Control the robot's turns    |
| Left joystick  | Control the robot's movement |

# References #

Here are some of the materials that we used to base and test our program