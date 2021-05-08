package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "TestingEncoders", group = "Testes")
public class TestingEncoders extends LinearOpMode {


    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;

    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 0.8;
    final double WHEEL_DIAMETER_CM = 10.16;


    @Override
    public void runOpMode() {


        //Find motors in the hub
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");


        //This offsets the two motors that are facing the opposite direction.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Stop and reset
        waitForStart();

        encoderDrive(50, 0.4);
        sleep(2000);

    }

    //method encoder drive forward
    public void encoderDrive ( int distance, double speed){

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);

        while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()) {
            telemetry.addData("Situação: ", "Andando");
            telemetry.addData("Posição/Rotação", FR.getCurrentPosition());
            telemetry.update();
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    //method encoder strafe to the left
    public void strafeDriveEsquerda(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(-targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(-targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);

    }

    //method encoder strafe right
    public void strafeDriveDireita(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(-targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(-targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }

    //method move diagonal to left forward
    public void moveDiagonalLeftFront(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        BL.setPower(speed);
    }

    //method move diagonal to rigth forward
    public void moveDiagonalRightFront(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(speed);
        BR.setPower(speed);
    }

    //method move diagonal to left back
    public void moveDiagonalLeftBack(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(-targetEncoder);
        BR.setTargetPosition(-targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(speed);
        BR.setPower(speed);
    }

    //method move diagonal to right back;
    public void moveDiagonalRightBack(int distance, double speed) {

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int) (rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(-targetEncoder);
        BR.setTargetPosition(-targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(speed);
        BR.setPower(speed);
    }

    private void resetEncoder () {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}




