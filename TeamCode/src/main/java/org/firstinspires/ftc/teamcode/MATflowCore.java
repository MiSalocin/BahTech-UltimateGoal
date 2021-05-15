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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name ="MATflowCore", group = "official")
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
    /* Pink HUB     = 1
         Motor FL/0 = Yellow
         Motor FR/1 = Blue
         Motor BL/2 = White
         Motor BR/3 = Green

       Blue HUB      = 2
         Arm motor/0 = Yellow
         Int motor/2 = Blue
         Sho motor/1 = White
    */

    //Encoder constants
    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 1.25;
    final double WHEEL_DIAMETER_CM = 10.16;

    // defines the constant multipliers to the PD
    final double kp = 1.5;
    final double kd = 1;
    final double k = 40;

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
        sleep(1000);
        moveArmAuto(true);


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

    public void resetEncoder() {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void goWhite() {
        double speed = 0.4;
        int whiteTape = 500; //Alpha
        int redTape = 110; //Based on RGB

        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        double p;
        double d;
        double pd;
        double currentAngle;
        double error;
        double lastError = 0;


        while (colorSensor.alpha() < whiteTape && colorSensor.red() < redTape) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;

            p = error * kp;
            d = (error - lastError) * kd;
            lastError = error;
            pd = (p + d) / k;

            FL.setPower(speed-pd);
            FR.setPower(speed+pd);
            BL.setPower(speed-pd);
            BR.setPower(speed+pd);
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    private void goZoneA(){

        //Delivery 1st wobble goal
        goWhite();
        moveArmAuto(false);
        sleep(1200);
        moveClawAuto(0);
        sleep(1500);
        moveArmAuto(true);
        sleep(1200);
        moveClawAuto(1);
        sleep(1000);
        turn(0.5, false, 36, 12);
        sleep(1000);

        //Go to the 2nd one
        movePD(-170, 0.5);
        sleep(500);
        moveArmAuto(false);
        moveClawAuto(0);
        sleep(500);
        movePD(10, 0.5);
        moveClawAuto(1);
        sleep(1000);
        moveArmAuto(true);

        //Go to the target zone with the 2nd wobble
        movePD(190, 0.5);
        moveArmAuto(false);
        sleep(1000);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(false);
        moveClawAuto(1);

    }

    private void goZoneB(){
        //Delivery the 1st wobble goal
        goWhite();
        turn(0.4, true, 35, 12);
        sleep(500);
        movePD(45, 0.4);
        moveArmAuto(false);
        moveClawAuto(0);
        sleep(1600);
        moveArmAuto(true);
        moveClawAuto(1);

        //Drive to the second one
        turn(0.5, false, 40, 12);
        movePD(-70, 0.5);
        moveClawAuto(0);
        moveArmAuto(false);
        sleep(1200);
        turn(0.5, true, 9, 3);
        sleep(1200);
        //driveForward(0.3,0.3);
        moveClawAuto(1);
        sleep(500);

        //Delivery the 2o one
        goWhite();
        sleep(500);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(true);
        moveClawAuto(1);
    }

    private void goZoneC(){
        goWhite();
        movePD(75, 0.4);
        moveArmAuto(false);
        sleep(1300);
        moveClawAuto(0);
        sleep(1600);
        moveArmAuto(true);
        moveClawAuto(1);

        //Drive to the second one
        turn(0.5, false, 40, 12);
        movePD(-110, 0.5);
        moveClawAuto(0);
        moveArmAuto(false);
        sleep(1200);
        turn(0.5, true, 9, 3);
        sleep(1200);
        //driveForward(0.3,0.3);
        moveClawAuto(1);
        sleep(500);

        //Delivery the 2o one
        goWhite();
        sleep(500);
        //driveForward(0.5, 0.5);
        moveClawAuto(0);
        sleep(1000);
        moveArmAuto(true);
        moveClawAuto(1);
    }

    /*MOVEMENT METHODS*/
    // Move the robot using PD
    private void movePD(double distance, double speed){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle;

        // define the PD variables
        double p;
        double d;
        double pd = 0 ;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
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

        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);

        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // the PID in action
            error = angle - currentAngle;

            p = error * kp;
            d = (error - lastError) * kd;
            lastError = error;
            pd = (p + d) / k;

            if (distance < 0){ pd *= -1; }

            telemetry.addData("PID VALUE", pd);

            FL.setPower(speed - pd);
            FR.setPower(speed + pd);
            BL.setPower(speed - pd);
            BR.setPower(speed + pd);

            telemetry.addData("FL power", FL.getPower());
            telemetry.addData("FR power", FR.getPower());
            telemetry.addData("BL power", BL.getPower());
            telemetry.addData("BR power", BR.getPower());

            telemetry.update();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Program used to precisely turn the robot
    public void turn(double force, boolean right, double targetAngle, int threshold) {
        double angle;
        final double smoother = 60;
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

    // Move the robot's arm
    private void moveArmAuto(boolean isUp ) {

        final int up = 700;
        final int down = -490;
        final double force = 0.7;

        if (isUp) {
            armMotor.setTargetPosition(up);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(-force);
        } else {
            armMotor.setTargetPosition(down);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(force);
        }

        if (!armMotor.isBusy()) {
            armMotor.setPower(0);
        }
    }

    // Open or close the arm's claw
    private void moveClawAuto(int requiredPosition) {
        clawServo.setPosition(requiredPosition);
    }

    /*LIBRARY INITIALIZER METHODS*/
    // Initialize the Vuforia Library
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the Tensor Flow Object Detection
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Initialize the REV Hub IMU
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
