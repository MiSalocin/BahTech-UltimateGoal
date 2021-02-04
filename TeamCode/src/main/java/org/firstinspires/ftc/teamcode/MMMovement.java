package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MMMovement {

    // Variables used in the shooter and the claw
    // private int    x = 0;
    // private double y = 0.8;

    // Motors that will be used in the movement
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    // Defining motor for claw and shooter
    // Motors that will be used in the claw
    // private DcMotor arm;
    // private Servo hand;

    // Motors that will be used in the shooter
    // private DcMotor shooter;
    // private Servo shooT;

    // Starts the IMU
    private BNO055IMU imu;
    private Acceleration gravity;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double z=0;
    private double inter = 1;
    private double exter = 1;

    void defHardware (HardwareMap local) {

        // Motors used in the movement
        frontLeft = local.dcMotor.get("front_left_motor");
        frontRight = local.dcMotor.get("front_right_motor");
        backLeft = local.dcMotor.get("back_left_motor");
        backRight = local.dcMotor.get("back_right_motor");

        // Reverse right motors to make programming easier
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*Specify the claw and shooter motors
         // Motors that wil be used in the claw
         arm = local.dcMotor.get("arm_motor");
         hand = local.servo.get("hand_servo");
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         // Motors that will be used in the shooter
         shooter = local.dcMotor.get("shooter_motor");
         shooT = local.servo.get("shooter_trig_servo");
         */
    }

    void moveRobot (double leftX, double leftY, double rightX, boolean slower, boolean faster){

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            frontRight.setPower(( + leftY + leftX - rightX) / 4);
            backRight .setPower(( + leftY - leftX - rightX) / 4);
            frontLeft .setPower(( + leftY - leftX + rightX) / 4);
            backLeft  .setPower(( + leftY + leftX + rightX) / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            frontRight.setPower( + leftY + leftX - rightX);
            backRight .setPower( + leftY - leftX - rightX);
            frontLeft .setPower( + leftY - leftX + rightX);
            backLeft  .setPower( + leftY + leftX + rightX);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            frontRight.setPower(( + leftY + leftX - rightX) / 2);
            backRight .setPower(( + leftY - leftX - rightX) / 2);
            frontLeft .setPower(( + leftY - leftX + rightX) / 2);
            backLeft  .setPower(( + leftY + leftX + rightX) / 2);
        }
    }

    void startIMU (HardwareMap local){
        imu = local.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        z++;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    String moveArena(double leftY, double leftX, double rightX, boolean slower, boolean faster){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;

        if (0 < angle && angle <= 90){
            inter = -(angle/45-1);
            exter = 1;
        }
        else if (90 < angle && angle <= 180){
            inter = -1;
            exter = -((angle-90)/45-1);
        }
        else if (-90 < angle && angle <= 0){
            inter = 1;
            exter = angle/45+1;
        }
        else if (-180 < angle && angle <= 90){
            inter = (angle+90)/45+1;
            exter = -1;
        }

        double frForce = leftY * inter - leftX * exter - rightX,
                brForce = leftY * exter + leftX * inter - rightX,
                flForce = leftY * exter + leftX * inter + rightX,
                blForce = leftY * inter - leftX * exter + rightX;

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            frontRight.setPower(frForce / 4);
            backRight .setPower(brForce / 4);
            frontLeft .setPower(flForce / 4);
            backLeft  .setPower(blForce / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            frontRight.setPower(frForce);
            backRight .setPower(brForce);
            frontLeft .setPower(flForce);
            backLeft  .setPower(blForce);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            frontRight.setPower(frForce / 2);
            backRight .setPower(brForce / 2);
            frontLeft .setPower(flForce / 2);
            backLeft  .setPower(blForce / 2);
        }

        return "Inter: " + inter +
                "\nExter: " + exter +
                "\nAngle: " + angle;
    }

    //  Program used in the shot and claw
    //  void moveClaw(boolean up, boolean down, boolean openHand, boolean closeHand){
    //
    //  // move up or down the claw
    //  if (up   && -1   >= arm.getCurrentPosition()) {
    //  arm.setTargetPosition(x+=5);
    //  arm.setPower(1);
    //  }else if (down && -653 <= arm.getCurrentPosition()) {
    //  arm.setTargetPosition(x-=5);
    //  arm.setPower(1);
    //  }else arm.setPower(0);
    //
    //  // Open or close the robot's "hand"
    //  if (openHand  && y < 1) hand.setPosition(y += 0.05);
    //  if (closeHand && y > 0) hand.setPosition(y -= 0.05);
    //
    //  }
    //
    //  void shot(boolean trigger, double force){
    //
    //  shooter.setPower(force);
    //  if (trigger) shooT.setPosition(1);
    //  else   shooT.setPosition(0);
    //
    // }
    //
    // public double getY() {
    //     return y;
    // }
}