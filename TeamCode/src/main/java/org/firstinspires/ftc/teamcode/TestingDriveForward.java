package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Program used to know if our robot is moving straigh or not

@Autonomous(name = "TestingDriveBack", group = "Testes")
public class TestingDriveForward extends LinearOpMode {

    DcMotor FL = null;
    DcMotor BL = null;
    DcMotor FR = null;
    DcMotor BR = null;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            FL.setPower(-0.5);
            FR.setPower(-0.5);
            BL.setPower(-0.5);
            BR.setPower(-0.5);
           }
        }
    }

