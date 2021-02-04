package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MMCoreByArena", group = "Bahtech")
public class MMCoreByArena extends LinearOpMode {

    // Map the hardware to autonomus and teleOp
    final MMMovement moveTele = new MMMovement();

    double x = 0.8, y = 0;

    @Override
    public void runOpMode() {
        // Create the Hardware Map in both our classes
        moveTele.defHardware(hardwareMap);
        moveTele.startIMU(hardwareMap);

        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.x)
                x = 0.75;
            if (gamepad1.y)
                x = 0.8;
            // Create variables from the control
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            telemetry.addData("LextX: ", leftX);
            telemetry.addData("LextY: ", leftY);
            telemetry.addData("RightX: ", rightX);

            if (gamepad1.back)
                y = 1;
            if (gamepad1.start)
                y = 2;
            if (y == 0)
                telemetry.addData("Aperte START para definir a frente a partir do Robô e BACK para a da arena", "");
            if (y == 1)
                telemetry.addData("", moveTele.moveArena(leftX, leftY, rightX, lb, rb));
            if (y == 2)
                moveTele.moveRobot(leftX, leftY, rightX, lb, rb);
            telemetry.update();
        }

    }
}