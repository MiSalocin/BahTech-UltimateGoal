package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TestTouch", group = "Tests")
public class TestTouch extends LinearOpMode {

    TouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("isPressed", touchSensor.isPressed());
            telemetry.addData("value", touchSensor.getValue());
            telemetry.update();
        }

    }
}
