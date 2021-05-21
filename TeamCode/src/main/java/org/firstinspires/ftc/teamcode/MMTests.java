package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tester", group = "BahTech")
public class MMTests extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();
        // MOTORS
        DcMotor FR;
        DcMotor FL;
        DcMotor BR;
        DcMotor BL;
        DcMotor shooterMotor;
        DcMotor intake;
        DcMotor armMotor;

        // SERVOS
        Servo shooterServo;
        Servo clawServo;

        // SENSORS
        ColorRangeSensor colorSensor;
        BNO055IMU imu;
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        String Errors;

        // Test the Movement motors
        try {
            Errors = "Movement OK";
            // Test the movement motors
            // Motors used in the movement
            FL = hardwareMap.dcMotor.get("FL");
            FR = hardwareMap.dcMotor.get("FR");
            BL = hardwareMap.dcMotor.get("BL");
            BR = hardwareMap.dcMotor.get("BR");

            // Left motor's reverse direction
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);

            // Configure the motors encoders
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Move the robot forward for one second
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Show the encoder data on screen
            telemetry.addData("Encoder FL", FL.getCurrentPosition());
            telemetry.addData("Encoder FR", FR.getCurrentPosition());
            telemetry.addData("Encoder BL", BL.getCurrentPosition());
            telemetry.addData("Encoder BR", BR.getCurrentPosition());
            telemetry.update();

            if (FL.getCurrentPosition() == 0 || FR.getCurrentPosition() == 0 || BL.getCurrentPosition() == 0 || BR.getCurrentPosition() == 0) {
                Errors = "UNPLUGGED ENCODER: ";
            }

            if (FL.getCurrentPosition() == 0) {
                Errors += "FL, ";
            }
            if (FR.getCurrentPosition() == 0) {
                Errors = "FR, ";
            }
            if (BL.getCurrentPosition() == 0) {
                Errors = "BL, ";
            }
            if (BR.getCurrentPosition() == 0) {
                Errors = "BR.";
            }

            throw new Exception(Errors);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the arm motor
        try {
            telemetry.clear();
            telemetry.addData("TESTING MOTOR", "ARM");
            telemetry.update();
            armMotor = hardwareMap.dcMotor.get("arm_motor");
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armMotor.setTargetPosition(700);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(-0.8);
            sleep(1000);

            armMotor.setTargetPosition(-300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.8);
            sleep(2000);

            telemetry.clear();
            telemetry.addData("arm motor", armMotor.getCurrentPosition());
            telemetry.update();
            sleep(2000);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the shooter motor
        try {
            shooterMotor = hardwareMap.dcMotor.get("shooter_motor");
            shooterMotor.setPower(1);
            sleep(1000);
            shooterMotor.setPower(0);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the intake motor
        try {
            telemetry.clear();
            telemetry.addData("TESTING MOTOR", "INTAKE");
            telemetry.update();
            sleep(2000);
            intake = hardwareMap.dcMotor.get("intake_motor");
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the shooter servo
        try {
            telemetry.clear();
            telemetry.addData("TESTING SERVO", "SHOOTER");
            telemetry.update();
            sleep(2000);
            shooterServo = hardwareMap.servo.get("trig_servo");
            shooterServo.setPosition(1);
            sleep(2000);
            shooterServo.setPosition(0);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the claw servo
        try {
            telemetry.clear();telemetry.addData("TESTING SERVO", "CLAW"); telemetry.update();
            sleep(2000);
            clawServo = hardwareMap.servo.get("claw_servo");
            clawServo.setPosition(1);
            sleep(1500);
            clawServo.setPosition(0);
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the Light, Distance and IMU sensors
        try {
            telemetry.clear();telemetry.addData("SENSORS", "COLOR, DISTANCE, IMU"); telemetry.update();
            sleep(2000);


            // Initialize the IMU
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
            colorSensor = hardwareMap.get(ColorRangeSensor.class, "sensor_color");

            while (time.milliseconds()<10000){
                telemetry.clear();
                telemetry.addData("DISTANCE ", colorSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("ALPHA    ", colorSensor.alpha());
                telemetry.addData("IMU ANGLE", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }
    }
}

