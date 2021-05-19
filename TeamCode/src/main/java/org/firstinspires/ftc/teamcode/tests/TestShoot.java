package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testShoot", group = "Testes")
public class TestShoot extends LinearOpMode {

    private DcMotor shooterMotor;
    private Servo shooterServo;

    @Override
    public void runOpMode() {

        shooterServo = hardwareMap.servo.get("trig_servo");
        shooterMotor = hardwareMap.dcMotor.get("shooter_motor");

        waitForStart();

        while (opModeIsActive()) {
            float triggerValueR = gamepad1.right_trigger;
            float triggerValueL = gamepad1.left_trigger;

            while(triggerValueL < 0.3) {
            shooterMotor.setPower(0.85);

                shooterServo.setPosition(0.3);
                sleep(300);
                shooterServo.setPosition(0.8);

            telemetry.addData("LeftTrigger", triggerValueL);
            telemetry.addData("RightTrigger", triggerValueR);
            telemetry.update();
            }
        }
    }
}

