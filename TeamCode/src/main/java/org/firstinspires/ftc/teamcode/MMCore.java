package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MMCore", group = "Bahtech")
public class MMCore extends LinearOpMode {

    // Map the hardware to autonomous and teleOp
    final MMMovement move = new MMMovement();

    @Override
    public void runOpMode() {

        // Create the Hardware Map and start IMU
        move.defHardware(hardwareMap);

        // Run once you press PLAY
        waitForStart();

        // Program used to define if it will base itself using the robot or the arena
        double y = 1;
        double shooterForce = 0;

        // Run repeatly after you press play
        while(opModeIsActive()){

            // Create variables for the control
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dPadLeft = gamepad1.dpad_left;

            if(dPadLeft && move.getShooterForce()>=0){
                shooterForce-=0.01;
            }else if(dpadRight && move.getShooterForce()<=1){
                shooterForce+=0.01;
            }
            move.setShooterForce(shooterForce);


            if (gamepad1.back)  y = 1;
            if (gamepad1.start) y = 2;

            if (y != 0) {
                telemetry.addData("LextX: ", leftX);
                telemetry.addData("LextY: ", leftY);
                telemetry.addData("RightX: ", rightX);
                // telemetry.addData("Shooter Force: ", move.getShooterForce());
                telemetry.addData("","");
                telemetry.addData("Front Left Motor: ", move.getFlforce());
                telemetry.addData("Front Right Motor: ", move.getFRforce());
                telemetry.addData("Back Left Motor: ", move.getBlforce());
                telemetry.addData("Back Right Motor: ", move.getBrforce());
                telemetry.addData("","");
                telemetry.addData("Intern: ", move.getIntern());
                telemetry.addData("Extern: ", move.getExtern());
            }

            if (y == 1) move.moveByArena(leftY, leftX, rightX, lb, rb);
            if (y == 2) move.MoveByRobot(leftY, leftX, rightX, lb, rb);
            if (y == 0) telemetry.addData("Press START to guide with the robot's vision and BACK to arena's vision", "");

            telemetry.update();
        }
    }
}