package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MMCore", group = "Bahtech")
public class MMCore extends LinearOpMode {

    // Map the hardware to autonomus and teleOp
    final MMMovement move = new MMMovement();
    double x = 0.8, y = 0;

    @Override
    public void runOpMode() {

        // Create the Hardware Map and start IMU
        move.defHardware(hardwareMap);
        move.startIMU();

        //DcMotor shooter = hardwareMap.dcMotor.get("shooter_motor");
        //Servo shooT = hardwareMap.servo.get("shooter_trig_servo");

        // Run once you press PLAY
        waitForStart();

        telemetry.addData("Aperte START para definir a frente a partir do Robô e BACK para a da arena", "");
        while (y==0) {
            if (gamepad1.back)  y = 1;
            if (gamepad1.start) y = 2;}

        // Run repeatly after you press play
        while(opModeIsActive()){
            telemetry.update();


            // Create variables from the control
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            telemetry.addData ("LextX: ", leftX);
            telemetry.addData ("LextY: ", leftY);
            telemetry.addData ("RightX: ", rightX);

            if (y == 0) telemetry.addData("", move.moveByArena(leftX, leftY, rightX, lb, rb));
            if (y == 2) move.MoveByRobot(leftX, leftY, rightX, lb, rb);

            //if (gamepad1.x) x = 0.75;
            //if (gamepad1.y) x = 0.8;
            //shooter.setPower(x);
            //if (gamepad1.a) shooT.setPosition(1);
            //else   shooT.setPosition(0);

        }
    }
}