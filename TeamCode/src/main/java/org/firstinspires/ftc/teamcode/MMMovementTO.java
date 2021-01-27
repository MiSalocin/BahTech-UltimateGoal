package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MMMovementTO {

    // Variables used in the shooter and the claw
    private int    x = 0;
    private double y = 1;

    // Motors that will be used in the movement
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    // Motors that will be used in the claw
    private DcMotor arm;
    private Servo hand;

    // Motors that will be used in the shooter
    private DcMotor shooter;
    private Servo shooT;

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

    void moveGarra(boolean up, boolean down, boolean openHand, boolean closeHand){

        // move up or down the claw
        if (up   && -1   >= arm.getCurrentPosition()) {
            arm.setTargetPosition(x+=5);
            arm.setPower(1);
        }else if (down && -653 <= arm.getCurrentPosition()) {
            arm.setTargetPosition(x-=5);
            arm.setPower(1);
        }else arm.setPower(0);

        // Open or close the "hand" of the claw
        if (openHand  && y < 1) hand.setPosition(y += 0.05);
        if (closeHand && y > 0) hand.setPosition(y -= 0.05);

    }

    void tiro(boolean trigger, double force){

        shooter.setPower(force);
        if (trigger) shooT.setPosition(1);
        else   shooT.setPosition(0);

    }

    public int getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}