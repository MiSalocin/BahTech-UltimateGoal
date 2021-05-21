package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "TestDistance", group = "Test")
public class TestDistance extends LinearOpMode{

    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("DISTANCE IN CM: ", sensorDistance.getDistance(DistanceUnit.CM));
        }
    }
}
