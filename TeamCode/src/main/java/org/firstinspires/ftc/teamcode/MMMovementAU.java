package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MMMovementAU {

    // Variables used in the shooter and the claw
    private int    x = 0;
    private double y = 0.8;

    // Motors that will be used in the movement
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor intake;

    /* Motors that will be used in the claw
    private DcMotor arm;
    private Servo hand;

    // Motors that will be used in the shooter
    private DcMotor shooter;
    private Servo shooT;
    */

    void defHardware (HardwareMap local) {

        // Motors used in the movement
        frontLeft = local.dcMotor.get("front_left_motor");
        frontRight = local.dcMotor.get("front_right_motor");
        backLeft = local.dcMotor.get("back_left_motor");
        backRight = local.dcMotor.get("back_right_motor");

        // Reverse right motors to make programming easier
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*
        // Motors that wil be used in the claw
        arm = local.dcMotor.get("arm_motor");
        hand = local.servo.get("hand_servo");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors that will be used in the shooter
        shooter = local.dcMotor.get("shooter_motor");
        shooT = local.servo.get("shooter_trig_servo");

        // Motors that will be on all the time
        intake = local.dcMotor.get("intake_motor");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(1);
         */
    }

    void moveF (double front, double side, double turn){

        // Move robot in full speed
        frontRight.setPower( + front + side - turn );
        backRight .setPower( + front - side - turn );
        frontLeft .setPower( + front - side + turn );
        backLeft  .setPower( + front + side + turn );

    }
    void moveS (double front, double side, double turn){

        // Move robot in a quart of speed
        frontRight.setPower(( + front + side - turn )/4);
        backRight .setPower(( + front - side - turn )/4);
        frontLeft .setPower(( + front - side + turn )/4);
        backLeft  .setPower(( + front + side + turn )/4);

    }
    void moveN (double front, double side, double turn){

        // Move robot in a half of speed
        frontRight.setPower(( + front + side - turn )/2);
        backRight .setPower(( + front - side - turn )/2);
        frontLeft .setPower(( + front - side + turn )/2);
        backLeft  .setPower(( + front + side + turn )/2);

    }
}