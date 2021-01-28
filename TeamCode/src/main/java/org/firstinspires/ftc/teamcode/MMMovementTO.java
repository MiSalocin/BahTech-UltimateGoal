package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MMMovementTO {

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
    **/

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

    void moveRobot (double leftX, double leftY, double rightX, boolean slower, boolean faster){

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            frontRight.setPower((+leftY + leftX - rightX) / 4);
            backRight .setPower((+leftY - leftX - rightX) / 4);
            frontLeft .setPower((+leftY - leftX + rightX) / 4);
            backLeft  .setPower((+leftY + leftX + rightX) / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            frontRight.setPower( + leftY + leftX - rightX );
            backRight .setPower( + leftY - leftX - rightX );
            frontLeft .setPower( + leftY - leftX + rightX );
            backLeft  .setPower( + leftY + leftX + rightX );
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            frontRight.setPower(( + leftY + leftX - rightX ) / 2);
            backRight .setPower(( + leftY - leftX - rightX ) / 2);
            frontLeft .setPower(( + leftY - leftX + rightX ) / 2);
            backLeft  .setPower(( + leftY + leftX + rightX ) / 2);
        }
    }

    /**Program used in the shot and claw
    void moveClaw(boolean up, boolean down, boolean openHand, boolean closeHand){

        // move up or down the claw
        if (up   && -1   >= arm.getCurrentPosition()) {
            arm.setTargetPosition(x+=5);
            arm.setPower(1);
        }else if (down && -653 <= arm.getCurrentPosition()) {
            arm.setTargetPosition(x-=5);
            arm.setPower(1);
        }else arm.setPower(0);

        // Open or close the robot's "hand"
        if (openHand  && y < 1) hand.setPosition(y += 0.05);
        if (closeHand && y > 0) hand.setPosition(y -= 0.05);

    }

    void shot(boolean trigger, double force){

        shooter.setPower(force);
        if (trigger) shooT.setPosition(1);
        else   shooT.setPosition(0);

    }
     */

    public double getY() {
        return y;
    }
}