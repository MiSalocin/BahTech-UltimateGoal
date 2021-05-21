package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "My Mechanum: Core", group = "BahTech")
public class MMCore extends LinearOpMode {
    final ElapsedTime time = new ElapsedTime();

    // Map the hardware to autonomous and TeleOp
    final MMMovement move = new MMMovement();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        // Create the Hardware Map and start IMU
        move.defHardware(hardwareMap);
        move.initIMU(hardwareMap);
        move.intakeForce(0);

        // Run once you press PLAY
        waitForStart();

        // Run repeatedly after you press play
        while(opModeIsActive()){

            if (gamepad1.dpad_up && move.getIntakeForce() != 0){
                move.intakeForce(0);
                sleep(250);
            }
            else if (gamepad1.dpad_up && move.getIntakeForce() != 0.8){
                move.intakeForce(1);
                sleep(250);
            }

            // Create variables for the control
            double leftX = -gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            move.shoot(gamepad1.right_trigger != 0.0);
            move.powerShot(gamepad1.left_trigger  != 0.0);

            move.move(leftY, leftX, rightX, lb, rb);
            move.claw(gamepad1.dpad_down, gamepad1.y, gamepad1.a, gamepad1.b, gamepad1.x);

            telemetry.addData("Front Left Motor",  String.format("%.2f", move.getFlForce()));
            telemetry.addData("Front Right Motor", String.format("%.2f", move.getFrForce()));
            telemetry.addData("Back Left Motor",   String.format("%.2f", move.getBlForce()));
            telemetry.addData("Back Right Motor",  String.format("%.2f", move.getBrForce()));
            telemetry.update();
        }
    }
}