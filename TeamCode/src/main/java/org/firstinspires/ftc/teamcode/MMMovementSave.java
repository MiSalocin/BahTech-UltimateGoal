package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MMMovementSave {
    // Abstract Variables
    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    private double internal = 1;
    private double external = 1;

    // Setting up motors variables
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private double robotFront;
    private double pd;
    private final ElapsedTime time = new ElapsedTime();
    private DcMotor shooterMotor;
    private Servo shooterServo;
    private DcMotor intake;
    private DcMotor armMotor;
    private Servo clawServo;

    // IMU variables
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // Program used to define the hardware variables
    public void defHardware(HardwareMap local) {

        // Define IMU
        imu = local.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        // Motors used in the movement
        FL = local.dcMotor.get("FL");
        FR = local.dcMotor.get("FR");
        BL = local.dcMotor.get("BL");
        BR = local.dcMotor.get("BR");

        clawServo = local.servo.get("claw_servo");
        armMotor = local.dcMotor.get("arm_motor");
        shooterServo = local.servo.get("trig_servo");
        shooterMotor = local.dcMotor.get("shooter_motor");
        intake = local.dcMotor.get("intake_motor");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Program used to move the robot using the arena
    public void moveByArena(double leftY, double leftX, boolean slower, boolean faster) {

        // Create a variable using the IMU
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Will see the IMU value and change the "internal" and "external" variables
        if (0 < angle && angle <= 90) {
            internal = -(angle / 45 - 1);
            external = 1;
        } else if (90 < angle && angle <= 180) {
            internal = -1;
            external = -((angle - 90) / 45 - 1);
        } else if (-90 < angle && angle <= 0) {
            internal = 1;
            external = angle / 45 + 1;
        } else if (-180 < angle && angle <= 90) {
            internal = (angle + 90) / 45 + 1;
            external = -1;
        }

        // Will start the normal "MoveByRobot" program, but using the Intern and Extern Multipliers
        moveByRobot(-leftY, -leftX, slower, faster);
    }

    // Program used to move the robot by himself
    // Define the same commands given in the MoveByArena
    public void moveByRobot(double leftY, double leftX, boolean slower, boolean faster) {
        final double smoother = 0.1;

        // Add to the vector the forces of the gamepad multiplying this with the angle defined
        // in the "MoveByArena" program.
        force[0] = leftY * internal + leftX * external + pd;
        force[1] = leftY * external - leftX * internal + pd;
        force[2] = leftY * external - leftX * internal - pd;
        force[3] = leftY * internal + leftX * external - pd;

        // See if the difference of the last force and the current one is bigger than 0.1,
        // if it is, it will change gradually to not damage the motors
        if (Math.abs(lastForce[0] - force[0]) > smoother) {
            if (lastForce[0] > force[0]) force[0] = lastForce[0] - smoother;
            else force[0] = lastForce[0] + smoother;
        }
        if (Math.abs(lastForce[1] - force[1]) > smoother) {
            if (lastForce[1] > force[1]) force[1] = lastForce[1] - smoother;
            else force[1] = lastForce[1] + smoother;
        }
        if (Math.abs(lastForce[2] - force[2]) > smoother) {
            if (lastForce[2] > force[2]) force[2] = lastForce[2] - smoother;
            else force[2] = lastForce[2] + smoother;
        }
        if (Math.abs(lastForce[3] - force[3]) > smoother) {
            if (lastForce[3] > force[3]) force[3] = lastForce[3] - smoother;
            else force[3] = lastForce[3] + smoother;
        }

        // Save the used force in variables to get the difference
        lastForce[0] = force[0];
        lastForce[1] = force[1];
        lastForce[2] = force[2];
        lastForce[3] = force[3];

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            FR.setPower(force[0] / 4);
            BR.setPower(force[1] / 4);
            FL.setPower(force[2] / 4);
            BL.setPower(force[3] / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            FR.setPower(force[0]);
            BR.setPower(force[1]);
            FL.setPower(force[2]);
            BL.setPower(force[3]);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            FR.setPower(force[0] / 2);
            BR.setPower(force[1] / 2);
            FL.setPower(force[2] / 2);
            BL.setPower(force[3] / 2);
        }
    }

    // Program used to turn the robot to one side.
    // define the angle in degrees, if it shout turn left or right and the force
    public void intakeForce(double force) {
        intake.setPower(force);
    }

    public void turn(double force, boolean right, double targetAngle) {
        double angle;
        final double smoother = 60;
        final int threshold = 7;
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (right) {
            angle = -targetAngle + currentAngle;
            angle -= threshold;
            if (angle < -180) {
                angle += 360;
            }
            if (angle - currentAngle > 180) {
                currentAngle += 360;
            }
            while (angle + threshold <= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - currentAngle > 180)
                    currentAngle += 360;
                if (force < ((angle - currentAngle) / smoother)) {
                    FR.setPower(force);
                    BR.setPower(force);
                    FL.setPower(-force);
                    BL.setPower(-force);
                } else {
                    FR.setPower((angle - currentAngle) / smoother);
                    BR.setPower((angle - currentAngle) / smoother);
                    FL.setPower(-(angle - currentAngle) / smoother);
                    BL.setPower(-(angle - currentAngle) / smoother);
                }
            }
        } else {
            angle = targetAngle + currentAngle;
            angle += threshold;
            if (angle > 180) {
                angle -= 360;
            }
            if (currentAngle - angle > 180)
                currentAngle -= 360;
            while (angle - threshold >= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (currentAngle - angle > 180)
                    currentAngle -= 360;
                if (force < ((currentAngle - angle) / smoother)) {
                    FR.setPower(-force);
                    BR.setPower(-force);
                    FL.setPower(force);
                    BL.setPower(force);
                } else {
                    FR.setPower(-(currentAngle - angle) / smoother);
                    BR.setPower(-(currentAngle - angle) / smoother);
                    FL.setPower((currentAngle - angle) / smoother);
                    BL.setPower((currentAngle - angle) / smoother);
                }
            }
        }
    }


    // Program used to turn the robot 180 degrees.
    // define the motors force
   /* public void inverter(double force) {
        // Calculate the objective degree
        double smoother = 25;
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (currentAngle < 0.4 && currentAngle > -0.4) {
            angle = 179.7;
        } else if (currentAngle > 0) {
            angle = -180 + currentAngle;
        } else {
            angle = 180 + currentAngle;
        }

        // Choose if it will turn right or left
        if (angle >= currentAngle) {
            while (angle >= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (force < ((angle - currentAngle) / smoother)) {
                    FR.setPower(-force);
                    BR.setPower(-force);
                    FL.setPower( force);
                    BL.setPower( force);
                } else {
                    FR.setPower(-(angle - currentAngle) / smoother);
                    BR.setPower(-(angle - currentAngle) / smoother);
                    FL.setPower( (angle - currentAngle) / smoother);
                    BL.setPower( (angle - currentAngle) / smoother);
                }
            }
        } else {
            while (angle <= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (force < ((currentAngle - angle) / smoother)) {
                    FR.setPower( force);
                    BR.setPower( force);
                    FL.setPower(-force);
                    BL.setPower(-force);
                } else {
                    FR.setPower(( currentAngle - angle) / smoother);
                    BR.setPower(( currentAngle - angle) / smoother);
                    FL.setPower(-(currentAngle - angle) / smoother);
                    BL.setPower(-(currentAngle - angle) / smoother);
                }
            }
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }*/

    // Program used to shoot the rings to the high goals/power shots
    public void shoot(boolean trigger) throws InterruptedException {
        shooterMotor.setPower(-1);
        if (trigger) {
            shooterServo.setPosition(1);
            wait(500);
        } else {
            shooterServo.setPosition(0);
        }
    }

    public void shootPowerShot(boolean trigger) throws InterruptedException {
        shooterMotor.setPower(-0.8);
        if (trigger) {
            shooterServo.setPosition(1);
            wait(500);

        } else {
            shooterServo.setPosition(0);
        }

    }


    // Program used to move the robot claw
    public void claw(boolean halfDown, boolean totalUp, boolean totalDown, boolean open, boolean close) {

        final int up = 700;
        final int down = -520;
        final int half = 300;
        final double force = 0.7;


        if (open) {
            clawServo.setPosition(1);
        } else if (close) {
            clawServo.setPosition(0);
        }

        if (totalUp){
            armMotor.setTargetPosition(up);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(-force);
        }
        else if (totalDown) {
            armMotor.setTargetPosition(down);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(force);
        }
        else if (halfDown) {
            armMotor.setTargetPosition(half);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(force);
        }

        if (!armMotor.isBusy()){
            armMotor.setPower(0);
        }
    }

    public void initIMU (HardwareMap local){
        imu = local.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void setPd(double rightX) {
        double currentX = 1;
        double currentY = 1;
        double x = 1;
        double y = 1;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        int objective = (int) (rightX * 5);

        if ((time.milliseconds() % 5) == 0) {
            if (rightX > 0.1) {
                if(robotFront > -179.9){
                    robotFront -= objective;
                } else {
                    robotFront += 359;
                }
            } else if (rightX < 0.1){
                if(robotFront > 179.9){
                    robotFront += objective;
                } else {
                    robotFront -= 359;
                }
            }
        }

        if (0 < currentAngle && currentAngle <= 90) {
            currentX = -(currentAngle / 45 - 1);
            currentY = 1;
        }
        else if (90 < currentAngle && currentAngle <= 180) {
            currentX = -1;
            currentY = -((currentAngle - 90) / 45 - 1);
        }
        else if (-90 < currentAngle && currentAngle <= 0) {
            currentX = 1;
            currentY = currentAngle / 45 + 1;
        }
        else if (-180 < currentAngle && currentAngle <= 90) {
            currentX = (currentAngle + 90) / 45 + 1;
            currentY = -1;
        }

        if (0 < robotFront && robotFront <= 90) {
            x = -(robotFront / 45 - 1);
            y = 1;
        }
        else if (90 < robotFront && robotFront <= 180) {
            x = -1;
            y = -((robotFront - 90) / 45 - 1);
        }
        else if (-90 < robotFront && robotFront <= 0) {
            x = 1;
            y = robotFront / 45 + 1;
        }
        else if (-180 < robotFront && robotFront <= 90) {
            x = (robotFront + 90) / 45 + 1;
            y = -1;
        }

        double p = 0;
        double d = 0;
        double error;
        final double kp = 1;
        final double kd = 1;
        final double k = 1;

        error = (currentX+currentY)-(x*y);

    }

    // Getters and setters
    public double getFlForce(){ return FL.getPower(); }
    public double getFrForce(){ return FR.getPower(); }
    public double getBlForce(){ return BL.getPower(); }
    public double getBrForce(){ return BR.getPower(); }
    public double getArmEncoder() { return armMotor.getCurrentPosition(); }
}