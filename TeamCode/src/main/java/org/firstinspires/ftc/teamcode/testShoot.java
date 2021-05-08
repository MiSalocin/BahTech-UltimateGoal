package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testShoot", group = "Testes")
public class testShoot extends LinearOpMode {

    private DcMotor shooterMotor;
    private Servo shooterServo;

    @Override
    public void runOpMode() {

        shooterServo = hardwareMap.servo.get("trig_servo");
        shooterMotor = hardwareMap.dcMotor.get("shooter_motor");

        waitForStart();

        while (opModeIsActive()) {
            shooterMotor.setPower(0.8);
            float triggerValueR = gamepad1.right_trigger;
            float triggerValueL = gamepad1.left_trigger;

            if (triggerValueR > 0.3) {
                shooterServo.setPosition(0.3); //aqui
                sleep(500);
                shooterServo.setPosition(0.8); //aqui
            }

            telemetry.addData("LeftTrigger", triggerValueL);
            telemetry.addData("RightTrigger", triggerValueR);
            telemetry.update();
        }
    }
}

