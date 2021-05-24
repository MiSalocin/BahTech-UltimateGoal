package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name ="Giro", group = "BahTech")
public class PIDTurn extends LinearOpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    final double kp = 1;
    final double ki = 0.3;
    final double kd = 3;
    final double k = 50;

    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() throws InterruptedException {

        initIMU();

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            turn(0.5, true, 90);
            telemetry.addData("turned", "right");
            telemetry.update();
            sleep(5000);
            turn(0.5, false, 90);
            telemetry.addData("turned", "left");
            telemetry.update();
            sleep(5000);
        }
    }

    public void turn(double pidK, boolean isRight, double targetAngle) {

        final double threshold = 0.5;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angle;

        double p = 0;
        double i = 0;
        double d = 0;
        double pid = 0 ;
        double error;
        double lastError = 0;


        if (isRight) {

            angle = -targetAngle + currentAngle;
            if (angle < -180) angle += 360;
            if (angle - currentAngle > 180) currentAngle += 360;

            while (Math.abs(angle-currentAngle) > threshold) {

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - currentAngle > 180) currentAngle += 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = (p+i+d) / -k;

                telemetry.addData("P", p);
                telemetry.addData("I", i);
                telemetry.addData("D", d);

                telemetry.addData("CA", currentAngle);
                telemetry.addData("TA", angle);
                telemetry.update();

                FL.setPower(+ pid * pidK);
                FR.setPower(- pid * pidK);
                BL.setPower(+ pid * pidK);
                BR.setPower(- pid * pidK);

                sleep(100);

                lastError=error;
            }

        } else {

            angle = targetAngle + currentAngle;
            if (angle > 180) angle -= 360;
            if (currentAngle - angle > 180) currentAngle -= 360;

            while (Math.abs(angle-currentAngle) > threshold) {

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (currentAngle - angle > 180) currentAngle -= 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = -(p+i+d) / k;

                telemetry.addData("P", p);
                telemetry.addData("I", i);
                telemetry.addData("D", d);

                telemetry.addData("CA", currentAngle);
                telemetry.addData("TA", angle);
                telemetry.update();

                FL.setPower(+ pid * pidK);
                FR.setPower(- pid * pidK);
                BL.setPower(+ pid * pidK);
                BR.setPower(- pid * pidK);

                sleep(100);

                lastError=error;
            }
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    public void initIMU (){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

}