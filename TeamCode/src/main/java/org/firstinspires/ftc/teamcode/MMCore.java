package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MMCore", group = "Bahtech")
public class MMCore extends LinearOpMode {

    // Map the hardware to autonomus and teleOp
    final MMMovementTO moveTele = new MMMovementTO();
    final MMMovementAU moveAuto = new MMMovementAU();

    double x = 1;
    int n = 0;

    @Override
    public void runOpMode() {
        // Create the Hardware Map in both our classes
        moveAuto.defHardware(hardwareMap);
        moveTele.defHardware(hardwareMap);
        telemetry.addData("Press play to start TeleOp Mode", "");

        waitForStart();
        while(opModeIsActive()){

            // Show the buttons on the screen
            telemetry.addData("Left  Stick  X: ", gamepad1.left_stick_x);
            telemetry.addData("Left  Stick  Y: ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick  X: ", gamepad1.right_stick_x);
            telemetry.addData("Force: ", x);
            telemetry.update();

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

            n++; telemetry.addData("PCA", n);

            moveTele.moveRobot(leftX, leftY, rightX, lb, rb);

        }

    }
}