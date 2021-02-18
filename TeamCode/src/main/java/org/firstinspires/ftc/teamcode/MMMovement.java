package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MMMovement {

    // Motors that will be used in the movement
    private DcMotor frontRight,
            frontLeft,
            backRight,
            backLeft;

    // Starts the IMU
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double inter = 1;
    private double exter = 1;


    // Create vectors to define the forces with less variables
    double flforce = 0;
    double frforce = 0;
    double blforce = 0;
    double brforce = 0;

    double lastflforce = 0;
    double lastfrforce = 0;
    double lastblforce = 0;
    double lastbrforce = 0;

    // Program used to define the hardware variables
    void defHardware (HardwareMap local) {

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
        frontLeft = local.dcMotor.get("front_left_motor");
        frontRight = local.dcMotor.get("front_right_motor");
        backLeft = local.dcMotor.get("back_left_motor");
        backRight = local.dcMotor.get("back_right_motor");

        // Reverse right motors to make programming easier
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    // Program used to move the robot using the arena
    void moveByArena(double leftY, double leftX, double rightX, boolean slower, boolean faster){

        // Create a variable using the IMU
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;

        // Will see the IMU value and change the "inter" and "exter" variaibles
        if      (   0 < angle && angle <= 90 )   {inter = -(angle/45-1);    exter = 1;}
        else if (  90 < angle && angle <= 180)   {inter = -1;               exter = -((angle-90)/45-1);}
        else if ( -90 < angle && angle <= 0  )   {inter = 1;                exter = angle/45+1;}
        else if (-180 < angle && angle <= 90 )   {inter = (angle+90)/45+1;  exter = -1;}

        // Will start the normal "MoveByRobot" program, but using the Inter and Exter Multipliers
        MoveByRobot(leftY, leftX, rightX, slower, faster);
    }

    // Program used to move the robot by himself
    void MoveByRobot (double leftY, double leftX, double rightX, boolean slower, boolean faster){


        // Add to the vector the forces of the gamepad multiplying this with the angle defined
        // in the "MoveByArena" program.
        flforce = leftY * inter - leftX * exter - rightX;
        frforce = leftY * exter + leftX * inter - rightX;
        blforce = leftY * exter + leftX * inter + rightX;
        brforce = leftY * inter - leftX * exter + rightX;

        // See if the diference of the last force and the current one is bigger than 0.1,
        // if it is, it will change gradually to not damage the motors
        if (Math.abs(lastflforce - flforce) > 0.15 || Math.abs(lastflforce - flforce) < 0.15){
            if (lastflforce > flforce)              flforce = lastflforce - 0.15;
            else                                    flforce = lastflforce + 0.15;}

        if (Math.abs(lastfrforce - frforce) > 0.15 || Math.abs(lastfrforce - frforce) < 0.15){
            if (lastfrforce > frforce)              frforce = lastfrforce - 0.15;
            else                                    frforce = lastfrforce + 0.15;}

        if (Math.abs(lastblforce - blforce) > 0.15 || Math.abs(lastblforce - blforce) < 0.15){
            if (lastblforce > blforce)              blforce = lastblforce - 0.15;
            else                                    blforce = lastblforce + 0.15;}

        if (Math.abs(lastbrforce - brforce) > 0.15 || Math.abs(lastbrforce - brforce) < 0.15){
            if (lastbrforce > brforce)              brforce = lastbrforce - 0.15;
            else                                    brforce = lastbrforce + 0.15;}

        // Save the used force in variables to get the diference
        lastflforce = flforce;
        lastfrforce = frforce;
        lastblforce = blforce;
        lastbrforce = brforce;

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            frontRight.setPower(flforce / 4);
            backRight .setPower(frforce / 4);
            frontLeft .setPower(blforce / 4);
            backLeft  .setPower(brforce / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            frontRight.setPower(flforce);
            backRight .setPower(frforce);
            frontLeft .setPower(blforce);
            backLeft  .setPower(brforce);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            frontRight.setPower(flforce / 2);
            backRight .setPower(frforce / 2);
            frontLeft .setPower(blforce / 2);
            backLeft  .setPower(brforce / 2);
        }
    }

    // Program used to turn the robot to one side. With this, you can
    // define the angle in degrees, if it shout turn left or right and the force
    void turn(double angle, double right, double force) {
        imu.initialize(parameters);
        double currentAngle = 0;
        if (right==1){
            while (-angle <= currentAngle) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = angles.firstAngle;

                if (force < (angle-currentAngle/100)) {
                    frontRight.setPower(-force);
                    backRight. setPower(-force);
                    frontLeft. setPower( force);
                    backLeft.  setPower( force);
                }else{
                    frontRight.setPower(-(angle-currentAngle)/100);
                    backRight. setPower(-(angle-currentAngle)/100);
                    frontLeft. setPower( (angle-currentAngle)/100);
                    backLeft.  setPower( (angle-currentAngle)/100);
                }


            }
        }
        else{
            while (angle >= currentAngle) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = angles.firstAngle;

                if (force < (angle-currentAngle/100)) {
                    frontRight.setPower( force);
                    backRight. setPower( force);
                    frontLeft. setPower(-force);
                    backLeft.  setPower(-force);
                }else{
                    frontRight.setPower( (angle-currentAngle)/100);
                    backRight. setPower( (angle-currentAngle)/100);
                    frontLeft. setPower(-(angle-currentAngle)/100);
                    backLeft.  setPower(-(angle-currentAngle)/100);
                }
            }
            frontRight.setPower(0);
            backRight .setPower(0);
            frontLeft .setPower(0);
            backLeft  .setPower(0);
        }
    }
}
