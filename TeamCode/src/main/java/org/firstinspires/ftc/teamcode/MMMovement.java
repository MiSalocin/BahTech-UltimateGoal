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
    // private DcMotor intake;
    // private Servo shooterServo;

    // Starts the IMU
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double intern = 1;
    private double extern = 1;

    double shooterForce = 1;

    // Create vectors to define the forces with less variables
    private double flforce = 0;
    private double frforce = 0;
    private double blforce = 0;
    private double brforce = 0;

    private double lastflforce = 0;
    private double lastfrforce = 0;
    private double lastblforce = 0;
    private double lastbrforce = 0;

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
        // intake = local.dcMotor.get("intake_motor");
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

        // Will see the IMU value and change the "inter" and "exter" variaibles
        if      (   0 < angle && angle <= 90 )   {
            intern = -(angle/45-1);    extern = 1;}
        else if (  90 < angle && angle <= 180)   {
            intern = -1;               extern = -((angle-90)/45-1);}
        else if ( -90 < angle && angle <= 0  )   {
            intern = 1;                extern = angle/45+1;}
        else if (-180 < angle && angle <= 90 )   {
            intern = (angle+90)/45+1;  extern = -1;}

        // Will start the normal "MoveByRobot" program, but using the Inter and Exter Multipliers
        MoveByRobot(leftY, leftX, rightX, slower, faster);
    }

    // Program used to move the robot by himself
    public void MoveByRobot (double leftY, double leftX, double rightX, boolean slower, boolean faster){


        // Add to the vector the forces of the gamepad multiplying this with the angle defined
        // in the "MoveByArena" program.
        flforce = leftY * intern + leftX * extern + rightX;
        frforce = leftY * extern - leftX * intern + rightX;
        blforce = leftY * extern - leftX * intern - rightX;
        brforce = leftY * intern + leftX * extern - rightX;

        // See if the difference of the last force and the current one is bigger than 0.1,
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

        // Save the used force in variables to get the difference
        lastflforce = flforce;
        lastfrforce = frforce;
        lastblforce = blforce;
        lastbrforce = brforce;

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            FR.setPower(flforce / 4);
            BR .setPower(frforce / 4);
            FL .setPower(blforce / 4);
            BL  .setPower(brforce / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            FR.setPower(flforce);
            BR .setPower(frforce);
            FL .setPower(blforce);
            BL  .setPower(brforce);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            FR.setPower(flforce / 2);
            BR .setPower(frforce / 2);
            FL .setPower(blforce / 2);
            BL  .setPower(brforce / 2);
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

    double getIntern(){return intern;}
    double getExtern(){return extern;}
    double getFlforce (){return FL.getPower();};
    double getFRforce (){return FR.getPower();};
    double getBlforce (){return BL.getPower();};
    double getBrforce (){return BR.getPower();};
    // double getShooterForce(){return shooterMotor.getPower();}
    void setShooterForce(double shooterForce){this.shooterForce = shooterForce;}
}
