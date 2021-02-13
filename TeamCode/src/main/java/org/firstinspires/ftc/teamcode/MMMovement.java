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

import java.util.Vector;

public class MMMovement {

    // Variables used in the shooter and the claw
    // private int    x = 0;
    // private double y = 0.8;

    // Motors that will be used in the movement
    private DcMotor frontRight,
            frontLeft,
            backRight,
            backLeft;

    // Defining motor for claw and shooter
    // Motors that will be used in the claw
    // private DcMotor arm;
    // private Servo hand;

    // Starts the IMU variables
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double inter = 1;
    private double exter = 1;

    final private Vector<Double> force = new Vector<>();
    final private Vector<Double> lastForce = new Vector<>();

    // Program used to define the hardware variables
    void defHardware (HardwareMap local) {

        // Define IMU
        imu = local.get(BNO055IMU.class, "imu");

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
         */
    }

    // Define the IMU parameter that will be used when it is used
    void startIMU (){
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    // Program used to move the robot using the arena
    double moveByArena(double leftY, double leftX, double rightX, boolean slower, boolean faster){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;

        if (0 < angle && angle <= 90)      {inter = -(angle/45-1); exter = 1;}
        else if (90 < angle && angle <= 180)    {inter = -1; exter = -((angle-90)/45-1);}
        else if (-90 < angle && angle <= 0)     {inter = 1; exter = angle/45+1;}
        else if (-180 < angle && angle <= 90)   {inter = (angle+90)/45+1; exter = -1;}

        MoveByRobot(leftY, leftX, rightX, slower, faster);
        return angle;
    }

    // Program used to move the robot by himself
    void MoveByRobot (double leftY, double leftX, double rightX, boolean slower, boolean faster){

        force.add(0, leftY * inter - leftX * exter - rightX);
        force.add(1, leftY * exter + leftX * inter - rightX);
        force.add(2, leftY * exter + leftX * inter + rightX);
        force.add(3, leftY * inter - leftX * exter + rightX);

        if (Math.abs(lastForce.get(0)-force.get(0)) > 0.25){
            if (lastForce.get(0) > force.get(0))    force.add(0, lastForce.get(0)-0.25);
            else                                    force.add(0, lastForce.get(0)+0.25);}
        if (Math.abs(lastForce.get(1)-force.get(1)) > 0.25){
            if (lastForce.get(1) > force.get(1))    force.add(1, lastForce.get(1)-0.25);
            else                                    force.add(1, lastForce.get(1)+0.25);}
        if (Math.abs(lastForce.get(2)-force.get(2)) > 0.25){
            if (lastForce.get(2) > force.get(2))    force.add(2, lastForce.get(2)-0.25);
            else                                    force.add(2, lastForce.get(2)+0.25);}
        if (Math.abs(lastForce.get(0)-force.get(0)) > 0.25){
            if (lastForce.get(3) > force.get(3))    force.add(3, lastForce.get(3)-0.25);
            else                                    force.add(3, lastForce.get(3)+0.25);}

        lastForce.add(0, force.get(0));
        lastForce.add(1, force.get(1));
        lastForce.add(2, force.get(2));
        lastForce.add(3, force.get(3));

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            frontRight.setPower(force.get(0) / 4);
            backRight .setPower(force.get(1) / 4);
            frontLeft .setPower(force.get(2) / 4);
            backLeft  .setPower(force.get(3) / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            frontRight.setPower(force.get(0));
            backRight .setPower(force.get(1));
            frontLeft .setPower(force.get(2));
            backLeft  .setPower(force.get(3));
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            frontRight.setPower(force.get(0) / 2);
            backRight .setPower(force.get(1) / 2);
            frontLeft .setPower(force.get(2) / 2);
            backLeft  .setPower(force.get(3) / 2);
        }
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
}