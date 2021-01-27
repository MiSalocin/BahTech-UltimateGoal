package org.firstinspires.ftc.teamcode;

//import the FTC libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test", group = "Bahtech")
public class MMLinearOpMode extends LinearOpMode {

    public void runOpMode() {
        telemetry.addData("Aperte START para começar", "");
        double x = 1;
        final MovementTO move = new MovementTO();
        move.defHardware(hardwareMap);
        telemetry.update();

        // Código que será executado quando apertarmos START.
        waitForStart();

        // Código que será executado após apertarmos START.
        while (opModeIsActive()) {
            // Mostra os botoes em uso na tela.
            telemetry.addData("Stick Esquerdo X", gamepad1.left_stick_x);
            telemetry.addData("Stick Esquerdo Y", gamepad1.left_stick_y);
            telemetry.addData("Stick Direito X", gamepad1.right_stick_x);
            telemetry.addData("Força", x);
            telemetry.update();

            if (gamepad1.x)
                x = 0.75;
            if (gamepad1.y)
                x = 0.8;

            // Crias variáveis a partir do controle.
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            boolean dpUp = gamepad1.dpad_up;
            boolean dpDown = gamepad1.dpad_down;
            boolean dpLeft = gamepad1.dpad_left;
            boolean dpRight = gamepad1.dpad_right;

            move.moveRobo(leftX, leftY, rightX, lb, rb);
            move.moveGarra(dpUp, dpDown, dpLeft, dpRight);
            move.tiro(gamepad1.a, x);
        }
    }
}