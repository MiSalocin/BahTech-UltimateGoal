package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.net.CookieHandler;
import java.util.List;

@Autonomous(name ="MATflowCore", group = "oficial")
public class MATflowCore extends LinearOpMode {

    //Hardware variables
    ColorSensor colorSensor;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor armMotor;
    Servo clawServo;
    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //Encoder constants
    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 0.8;
    final double WHEEL_DIAMETER_CM = 10.16;

    //Detection variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    Recognition recognition;

    private final ElapsedTime runtime = new ElapsedTime();
    public static final String VUFORIA_KEY =
            "Ad0n+XX/////AAABmQ/41s5hKkoQl9XvVGzFatosnvXWi3lcaK406bSM6BCiUoPYCNo83nOVmmi0PcL1v6+gDcdnZddtNmRaYSKGqMhsoHzczhJbbzJa6vsQPZ6Bzzs/9icQySfGBy4wUXNFBysun4H4G2qQEeaWP/PwNu7FkREo28S5DXYbk5G1SToOk/6MiOG4v8zuy1WvA2Mrg2gbJMlj181zI7Wj+CYJXDUpE2ugflgq9iDDzBCJGb0r1/sEwkqpBq/u3G8F2iPK5kRxLSsz2yB8AjpiLZlQJoZ9NpIwkEyuS9i6AVc9lnPTMst+gortmeJ+ThBBhscsJ55tdDf8yLn11cQgJSyrWBr9+9HIyvRc3ixR/og6y/Mc";

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Inicializando");
        telemetry.update();

        List<Recognition> recognitions;
        double index;

        //Find motors in the hub
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        initIMU();

        //Wobble grabber
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        //Left motor's reverse direction
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        //Set motors zero power behavior
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        moveClawAuto(1);

        //Initialize Vuforia and TensorFlow
        initVuforia();
        initTfod();

        //Show a message to press play
        telemetry.addData(">", "Press Play to start!");
        telemetry.update();


        waitForStart();

        if (tfod != null) {
            tfod.activate();
        } //Activate TensorFlow

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                sleep(3000);
                recognitions = tfod.getRecognitions();

                if (recognitions.size() == 0) {
                    telemetry.addData("TFLOW", "No object detected");
                    telemetry.addData("Zona Alvo", "A");
                    goZoneA();
                } else {

                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                    }

                    if (recognition.getLabel().equals("Single")) {
                        telemetry.addData("Zona Alvo", "B");
                        goZoneB();
                        break;
                    } else if (recognition.getLabel().equals("Quad")) {
                        telemetry.addData("Zona Alvo", "C");
                        goZoneC();
                        break;
                    }
                }
                telemetry.update();
            }
        }

        if (tfod != null) {
            tfod.shutdown();

        }

    }

    //Class to encoder move straight
    public void encoderDrive(int distance, double speed){

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move

        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);

        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ){
            telemetry.addData("Situação: ", "Andando");
            telemetry.addData("Posição/Rotação", FR.getCurrentPosition());
            telemetry.update();
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void driveBack(double timePerSec, double speed) {

        runtime.reset();
        speed *=-1;

        while(runtime.seconds() < timePerSec) {
            FL.setPower(speed);
            FR.setPower(-5.5);
            BL.setPower(speed);
            BR.setPower(-5.5);
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void driveForward(double timePerSec, double speed) {
        final double smoother = 25;
        double force = (runtime.milliseconds() - timePerSec*(1000) / smoother);
        runtime.reset();

        while(runtime.seconds() < timePerSec) {
            if (speed < force) {
                FR.setPower(speed);
                BR.setPower(speed);
                FL.setPower(speed);
                BL.setPower(speed);
            } else {
                FR.setPower(-force);
                BR.setPower(-force);
                FL.setPower(-force);
                BL.setPower(-force);
            }
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void moveArmAuto(double speed, double timePerSec){
        runtime.reset();

        while (runtime.seconds() < timePerSec){
            armMotor.setPower(speed);
        }
        armMotor.setPower(0);
    }

    private void moveClawAuto(int requiredPosition) {
        clawServo.setPosition(requiredPosition);
    }

    private void resetEncoder() {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void goToWhite() {
        int whiteTape = 500; //Alpha
        int redTape = 110; //Based on RGB

        while (colorSensor.alpha() < whiteTape && colorSensor.red() < redTape) {

            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.update();

            FL.setPower(0.5);
            BL.setPower(0.5);
            FR.setPower(0.55);
            BR.setPower(0.55);
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    private void goZoneA(){
        //Delivery 1st wobble goal
        goToWhite();
        sleep(1500);
        moveArmAuto(-0.4, 1);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(0.5, 1.2);
        moveClawAuto(1);
        sleep(1000);
        turn(0.5, false, 36);
        sleep(1000);

        //Go to the 2nd one
        driveBack(3.4, 0.5);
        moveClawAuto(0);
        moveArmAuto(-0.5, 1.2);
        sleep(1200);
        turn(0.5, true, 9);
        sleep(1200);
        driveForward(0.3,0.3);
        moveClawAuto(1);
        moveArmAuto(0.6, 1.4);
        sleep(1200);

        //Go to the target zone with the 2nd wobble
        goToWhite();
        sleep(500);
        driveForward(0.5, 0.5);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(0.5, 1);
        moveClawAuto(1);
    }

    private void goZoneB(){
        //Delivery the 1st wobble goal
        goToWhite();
        turn(0.4, true, 35);
        sleep(500);
        encoderDrive(45, 0.4);
        moveArmAuto(-0.5, 1);
        moveClawAuto(0);
        sleep(1600);
        moveArmAuto(0.5, 1);
        moveClawAuto(1);

        //Drive to the second one
        turn(0.5, false, 40);
        encoderDrive(-70, 0.5);
        moveClawAuto(0);
        moveArmAuto(-0.5, 1.2);
        sleep(1200);
        turn(0.5, true, 9);
        sleep(1200);
        driveForward(0.3,0.3);
        moveClawAuto(1);
        sleep(500);

        //Delivery the 2o one
        goToWhite();
        sleep(500);
        driveForward(0.5, 0.5);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(0.5, 1);
        moveClawAuto(1);
    }

    private void goZoneC(){
        goToWhite();
        encoderDrive(75, 0.4);
        moveArmAuto(-0.4, 1);
        sleep(1300);
        moveClawAuto(0);
        sleep(1600);
        moveArmAuto(0.5, 1);
        moveClawAuto(1);

        //Drive to the second one
        turn(0.5, false, 40);
        encoderDrive(-110, 0.5);
        moveClawAuto(0);
        moveArmAuto(-0.5, 1.2);
        sleep(1200);
        turn(0.5, true, 9);
        sleep(1200);
        driveForward(0.3,0.3);
        moveClawAuto(1);
        sleep(500);

        //Delivery the 2o one
        goToWhite();
        sleep(500);
        driveForward(0.5, 0.5);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(0.5, 1);
        moveClawAuto(1);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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


    public void movePID(double angle, double distance, double speed){
        runtime.reset();

        // defines the constant multipliers to the PID
        final double kp = 1;
        final double ki = 0.5;
        final double kd = 1;
        final double k = 100;
        final double updateRate = 1;

        // define the PID variables
        double p;
        double i = 0;
        double d;
        double pid = 0 ;
        double currentAngle;
        double error;
        double lastError = 0;

        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // the PID in action
            if (runtime.milliseconds() % updateRate == 0) {
                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                lastError = error;
                pid = (p + i + d) / k;
            }

            FL.setPower(speed-pid);
            FR.setPower(speed+pid);
            BL.setPower(speed-pid);
            BR.setPower(speed+pid);

        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void turn(double force, boolean right, double targetAngle) {
        double angle;
        final double smoother = 60;
        final int threshold = 9;
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (right) {
            angle = -targetAngle + currentAngle;
            angle -= threshold;
            if (angle < -180) {
                angle += 360;
            }
            if (angle - currentAngle > 180) {
                currentAngle += 360;
            }
            while (angle + threshold <= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - currentAngle > 180)
                    currentAngle += 360;
                if (force < -((angle - currentAngle) / smoother)) {
                    FR.setPower(-force);
                    BR.setPower(-force);
                    FL.setPower(force);
                    BL.setPower(force);
                } else {
                    FR.setPower((angle - currentAngle) / smoother);
                    BR.setPower((angle - currentAngle) / smoother);
                    FL.setPower(-(angle - currentAngle) / smoother);
                    BL.setPower(-(angle - currentAngle) / smoother);
                }
            }
        } else {
            angle = targetAngle + currentAngle;
            angle += threshold;
            if (angle > 180) {
                angle -= 360;
            }
            if (currentAngle - angle > 180)
                currentAngle -= 360;
            while (angle - threshold >= currentAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (currentAngle - angle > 180)
                    currentAngle -= 360;
                if (force < -((currentAngle - angle) / smoother)) {
                    FR.setPower(force);
                    BR.setPower(force);
                    FL.setPower(-force);
                    BL.setPower(-force);
                } else {
                    FR.setPower(-(currentAngle - angle) / smoother);
                    BR.setPower(-(currentAngle - angle) / smoother);
                    FL.setPower((currentAngle - angle) / smoother);
                    BL.setPower((currentAngle - angle) / smoother);
                }
            }
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }
}
