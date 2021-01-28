package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MMMovementAU {

    // Variables used in the shooter and the claw
    private int    x = 0;
    private double y = 0.8;

    // Motors that will be used in the movement
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    /** Defining motor for claw and shooter
     // Motors that will be used in the claw
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

        /*Specify the claw and shooter motors
         // Motors that wil be used in the claw
         arm = local.dcMotor.get("arm_motor");
         hand = local.servo.get("hand_servo");
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         // Motors that will be used in the shooter
         DcMotor intake = local.dcMotor.get("intake_motor");
         shooter = local.dcMotor.get("shooter_motor");
         shooT = local.servo.get("shooter_trig_servo");

        // Motors that will be on all the time
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

    /** Program used in the shot and claw
    void moveClaw(String Vertical, boolean openHand, boolean closeHand){

        // move up or down the claw
        if (Vertical.equals("up") || Vertical.equals("Up")) {
            arm.setTargetPosition(x = -1);
            arm.setPower(1);
        }else if (Vertical.equals("down") || Vertical.equals("Down")) {
            arm.setTargetPosition(x = -653);
            arm.setPower(1);
        }else arm.setPower(0);

        // Open or close the robot's "hand"
        if (openHand  && y < 1) hand.setPosition(y += 0.05);
        if (closeHand && y > 0) hand.setPosition(y -= 0.05);

    }

    void shot(double force){

        shooter.setPower(force);
        shooT.setPosition(1);
        shooT.setPosition(0);

    }
     */

    public double getY() {
        return y;
    }
}