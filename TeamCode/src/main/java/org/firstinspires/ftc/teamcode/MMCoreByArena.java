package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MMCoreByArena", group = "Bahtech")
public class MMCoreByArena extends LinearOpMode {

    // Map the hardware to autonomus and teleOp
    final MMMovementTO moveTele = new MMMovementTO();
    final MMMovementAU moveAuto = new MMMovementAU();

    double x = 1;

    @Override
    public void runOpMode() {
        // Create the Hardware Map in both our classes
        moveAuto.defHardware(hardwareMap);
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

            telemetry.addData("",moveTele.moveArena(leftX, leftY, rightX, lb, rb));
            telemetry.update();
        }

    }
}