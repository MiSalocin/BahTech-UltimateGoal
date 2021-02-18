package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MMCore", group = "Bahtech")
public class MMCore extends LinearOpMode {

    // Map the hardware to autonomus and teleOp
    final MMMovement move = new MMMovement();

    @Override
    public void runOpMode() {

        // Create the Hardware Map and start IMU
        move.defHardware(hardwareMap);
        move.turn(90, 1, 0.9);
        move.turn(90,0, 0.6);
        // Run once you press PLAY
        waitForStart();

        // Program used to define if it will base itself using the robot or the arena
        double y = 0;

        // Run repeatly after you press play
        while(opModeIsActive()){

            if (gamepad1.back)  y = 1;
            if (gamepad1.start) y = 2;

            // Create variables from the control
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (y != 0) telemetry.addData ("LextX: ", leftX);
            if (y != 0) telemetry.addData ("LextY: ", leftY);
            if (y != 0) telemetry.addData ("RightX: ", rightX);

            if (y == 1) move.moveByArena(leftX, leftY, rightX, lb, rb);
            if (y == 2) move.MoveByRobot(leftX, leftY, rightX, lb, rb);
            if (y == 0) telemetry.addData("Aperte START para definir a frente a partir do ",
                    "Robô e BACK para a da arena");

            telemetry.update();
        }
    }
}