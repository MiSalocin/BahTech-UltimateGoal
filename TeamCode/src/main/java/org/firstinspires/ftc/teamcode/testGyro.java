package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TestGiro", group = "BahTech")
public class testGyro extends LinearOpMode {

    MMMovement move = new MMMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        move.defHardware(hardwareMap);
        move.turn(0.1, true, 90);
        move.turn(0.1, true, 45);
        move.turn(0.1, false, 45);
        move.turn(0.1, false, 90);
    }
}
