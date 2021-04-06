package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MMMovement {

    // Motors that will be used in the movement
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;

    // private DcMotor shooterMotor;
    // private Servo shooterServo;

    // Starts the IMU
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double intern = 1;
    private double extern = 1;

    private double shooterForce = 1;

    // Create vectors to define the forces with less variables
    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    // Program used to define the hardware variables
    public void defHardware (HardwareMap local) {

        // Define IMU
        imu = local.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Motors used in the movement
        FL = local.dcMotor.get("front_left_motor");
        FR = local.dcMotor.get("front_right_motor");
        BL = local.dcMotor.get("back_left_motor");
        BR = local.dcMotor.get("back_right_motor");

        // shooterServo = local.servo.get("shooter_trig_servo");
        // shooterMotor = local.dcMotor.get("shooter_motor");
        // DcMotor intake = local.dcMotor.get("intake_motor");
        // shooterMotor.setPower(1);
        // intake.setPower(1);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Program used to move the robot using the arena
    public void moveByArena(double leftY, double leftX, double rightX, boolean slower, boolean faster){

        // Create a variable using the IMU
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;

        // Will see the IMU value and change the "intern" and "extern" variables
        if      (   0 < angle && angle <= 90 )   {
            intern = -(angle/45-1);    extern = 1;}
        else if (  90 < angle && angle <= 180)   {
            intern = -1;               extern = -((angle-90)/45-1);}
        else if ( -90 < angle && angle <= 0  )   {
            intern = 1;                extern = angle/45+1;}
        else if (-180 < angle && angle <= 90 )   {
            intern = (angle+90)/45+1;  extern = -1;}

        // Will start the normal "MoveByRobot" program, but using the Intern and Extern Multipliers
        MoveByRobot(leftY, leftX, rightX, slower, faster);
    }

    // Program used to move the robot by himself
    public void MoveByRobot (double leftY, double leftX, double rightX, boolean slower, boolean faster){
        final double smoother = 0.10;

        // Add to the vector the forces of the gamepad multiplying this with the angle defined
        // in the "MoveByArena" program.
        force[0] = leftY * intern + leftX * extern + rightX;
        force[1] = leftY * extern - leftX * intern + rightX;
        force[2] = leftY * extern - leftX * intern - rightX;
        force[3] = leftY * intern + leftX * extern - rightX;

        // See if the difference of the last force and the current one is bigger than 0.1,
        // if it is, it will change gradually to not damage the motors
        if (Math.abs(lastForce[0] - force[0]) > smoother){
            if (lastForce[0] > force[0])              force[0] = lastForce[0] - smoother;
            else                                    force[0] = lastForce[0] + smoother;}

        if (Math.abs(lastForce[1] - force[1]) > smoother){
            if (lastForce[1] > force[1])              force[1] = lastForce[1] - smoother;
            else                                    force[1] = lastForce[1] + smoother;}

        if (Math.abs(lastForce[2] - force[2]) > smoother){
            if (lastForce[2] > force[2])              force[2] = lastForce[2] - smoother;
            else                                    force[2] = lastForce[2] + smoother;}

        if (Math.abs(lastForce[3] - force[3]) > smoother){
            if (lastForce[3] > force[3])              force[3] = lastForce[3] - smoother;
            else                                    force[3] = lastForce[3] + smoother;}

        // Save the used force in variables to get the difference
        lastForce[0] = force[0];
        lastForce[1] = force[1];
        lastForce[2] = force[2];
        lastForce[3] = force[3];

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            FR.setPower(force[0] / 4);
            BR .setPower(force[1] / 4);
            FL .setPower(force[2] / 4);
            BL  .setPower(force[3] / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            FR.setPower(force[0]);
            BR .setPower(force[1]);
            FL .setPower(force[2]);
            BL  .setPower(force[3]);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            FR.setPower(force[0] / 2);
            BR .setPower(force[1] / 2);
            FL .setPower(force[2] / 2);
            BL  .setPower(force[3] / 2);
        }
    }

    // Program used to turn the robot to one side. With this, you can
    // define the angle in degrees, if it shout turn left or right and the force
    public void turn(double angle, double right, double force) {
        imu.initialize(parameters);
        double currentAngle = 0;
        if (right==1){
            while (-angle <= currentAngle) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = angles.firstAngle;

                if (force < (angle-currentAngle/100)) {
                    FR.setPower(-force);
                    BR. setPower(-force);
                    FL. setPower( force);
                    BL.  setPower( force);
                }else{
                    FR.setPower(-(angle-currentAngle)/100);
                    BR. setPower(-(angle-currentAngle)/100);
                    FL. setPower( (angle-currentAngle)/100);
                    BL.  setPower( (angle-currentAngle)/100);
                }
            }
        }
        else{
            while (angle >= currentAngle) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = angles.firstAngle;

                if (force < (angle-currentAngle/100)) {
                    FR.setPower( force);
                    BR. setPower( force);
                    FL. setPower(-force);
                    BL.  setPower(-force);
                }else{
                    FR.setPower( (angle-currentAngle)/100);
                    BR. setPower( (angle-currentAngle)/100);
                    FL. setPower(-(angle-currentAngle)/100);
                    BL.  setPower(-(angle-currentAngle)/100);
                }
            }
            FR.setPower(0);
            BR .setPower(0);
            FL .setPower(0);
            BL  .setPower(0);
        }
    }

    // Program used to shoot the rings to the high goals/power shots
    // public void shoot(boolean trigger) throws InterruptedException {
    //     shooterMotor.setPower(shooterForce);
    //     if (trigger){
    //         shooterServo.setPosition(1);
    //         wait(500);
    //     }else{
    //         shooterServo.setPosition(0);
    //     }
    // }

    public double getIntern(){return intern;}
    public double getExtern(){return extern;}
    public double getFlForce(){return FL.getPower();}
    public double getFrForce(){return FR.getPower();}
    public double getBlForce(){return BL.getPower();}
    public double getBrForce(){return BR.getPower();}
    // public double getShooterForce(){return shooterMotor.getPower();}
    // public void setShooterForce(double shooterForce){this.shooterForce = shooterForce;}
}
