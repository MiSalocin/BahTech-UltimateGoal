package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Thread.sleep;

@Autonomous(name ="MATflowCore", group = "official", preselectTeleOp = "MMCore")
public class MATflowCore extends LinearOpMode {

    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    // Hardware variables
    ColorSensor colorSensor;
    TouchSensor touchSensor;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // Secondary motors
    DcMotor armMotor;
    DcMotor intakeMotor;
    DcMotor shooterMotor;

    // Servos
    Servo clawServo;
    Servo trigServo;


    // Encoder constants
    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 1.25;
    final double WHEEL_CIRCUMFERENCE_CM = Math.PI*10.16;

    // Arm variables
    int up = 0;
    int down = -1000;

    // Defines the constant multipliers to the PID
    final double kp = 1.2;
    final double ki = 0.3;
    final double kd = 2;
    final double k = 35;
    final long updateRate = 100L;
    double angle = 0;

    //Detection variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    Recognition recognition;
    private final ElapsedTime runtime = new ElapsedTime();

    // Vuforia key
    public static final String VUFORIA_KEY =
            "Ad0n+XX/////AAABmQ/41s5hKkoQl9XvVGzFatosnvXWi3lcaK406bSM6BCiUoPYCNo83nOVmmi0PcL1v6+gDcdnZddtNmRaYSKGqMhsoHzczhJbbzJa6vsQPZ6Bzzs/9icQySfGBy4wUXNFBysun4H4G2qQEeaWP/PwNu7FkREo28S5DXYbk5G1SToOk/6MiOG4v8zuy1WvA2Mrg2gbJMlj181zI7Wj+CYJXDUpE2ugflgq9iDDzBCJGb0r1/sEwkqpBq/u3G8F2iPK5kRxLSsz2yB8AjpiLZlQJoZ9NpIwkEyuS9i6AVc9lnPTMst+gortmeJ+ThBBhscsJ55tdDf8yLn11cQgJSyrWBr9+9HIyvRc3ixR/og6y/Mc";

    @Override
    public void runOpMode() throws InterruptedException {


        List<Recognition> recognitions;

        // Find motors in the hub
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Wobble grabber
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        // Servos
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        trigServo = hardwareMap.get(Servo.class, "trig_servo");

        // Sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        // Left motor's reverse direction
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        // Set motors zero power behavior
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize IMU, Vuforia and TensorFlow. Calibrate the arm motor
        initIMU();
        calibrateArm();
        initVuforia();
        initTfod();

        // Lift the arm and close the claw
        moveArmAuto(true);
        sleep(1500);
        moveClawAuto(false);

        // Show a message to press play
        telemetry.addData(">", "Press Play to start!");
        telemetry.update();
        waitForStart();

        // Activate TensorFlow
        if (tfod != null) {
            tfod.activate();
        }

        // Start the programs to the determinate zone
        while (opModeIsActive()) {
            sleep(1500);

            // Get the size of the initial ring stack
            recognitions = tfod.getRecognitions();

            if (tfod != null) {
                tfod.shutdown();
            }

            if (recognitions.size() == 0) {
                telemetry.addData("TFLOW", "No object detected");
                telemetry.addData("Zona Alvo", "A");
                telemetry.update();
                goZoneA();
                break;
            } else {

                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                }

                if (recognition.getLabel().equals("Single")) {
                    telemetry.addData("Zona Alvo", "B");
                    telemetry.update();
                    goZoneB();
                    break;
                } else if (recognition.getLabel().equals("Quad")) {
                    telemetry.addData("Zona Alvo", "C");
                    telemetry.update();
                    goZoneC();
                    break;
                }
            }
            telemetry.update();
        }
    }

    // Program used to reset the movement encoders
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

    // Move the robot to the white tape
    private void goWhite() {
        double speed = 0.8;
        int whiteTape = 500; //Alpha
        int redTape = 110;   //Based on RGB
        double p;
        double currentAngle;
        double error;

        while (colorSensor.alpha() < whiteTape && colorSensor.red() < redTape) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;

            p = (error * kp) / k;

            FL.setPower(speed-p);
            FR.setPower(speed+p);
            BL.setPower(speed-p);
            BR.setPower(speed+p);
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

//    private void goBlue() {
//        double speed = 0.7;
//        int blueTape;
//        int redTape;
//
//        double p;
//        double currentAngle;
//        double error;
//
//        while (colorSensor.alpha() < blueTape && colorSensor.red() < redTape) {
//            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
//            error = angle - currentAngle;
//
//            p = (error * kp) / k;
//
//            FL.setPower(speed-p);
//            FR.setPower(speed+p);
//            BL.setPower(speed-p);
//            BR.setPower(speed+p);
//        }
//
//        FL.setPower(0);
//        BL.setPower(0);
//        FR.setPower(0);
//        BR.setPower(0);
//    }

    // Program run when there's no rings in front of the robot
    private void goZoneA() throws InterruptedException {

        //Delivery 1st wobble goal
        goWhite();
        moveArmAuto(false);
        sleep(1000);
        moveClawAuto(true);
        sleep(100);
        moveArmAuto(true);
        sleep(500);
        moveClawAuto(false);

        // Align to the power shoot position
        shooterMotor.setPower(0.725);
        movePID(-35, 0.8);
        movePIDSide(95, 0.7, true);
        shootPowerShoots(3);
        turn(0.75,false,12);

        // Land over the launch line
        goWhite();
    }

    // Program run when there's two rings in front of the robot
    private void goZoneB() throws InterruptedException {

        // Align to the starter stack by strafing right
        movePIDSide(35, 0.7, true);
        shooterMotor.setPower(0.78);
        movePID(55, 0.8);

        // Shoot in the high goal and lift the arm
        sleep(500);
        shootRing(1);
        moveArmAuto(true);

        // Turn on the intake and take the ring
        intakeMotor.setPower(0.8);
        movePID(40, 0.6);
        movePID(25, 0.8);
        sleep(500);

        // Align to the power shoot
        shooterMotor.setPower(0.725);
        intakeMotor.setPower(0);
        movePIDSide(50, 0.7, true);

        // Shoot power power shots
        shootPowerShoots(3);
        moveArmAuto(true);
        turn(0.75, false, 12);

        // Deliver the first Wobble Goal
        goWhite();
        movePIDSide(25,0.7, false);
        movePID(55,0.8);
        moveArmAuto(false);
        sleep(1000);
        moveClawAuto(true);
        sleep(500);
        moveArmAuto(true);
        movePID(-40,0.8);
    }

    // Program run when there's four rings in front of the robot
    private void goZoneC() throws InterruptedException {

        // Align to the starter stack by strafing right
        movePIDSide(35, 0.8, true);
        shooterMotor.setPower(0.8);
        movePID(50, 0.8);

        // Shoot in the high goal and lift the arm
        sleep(500);
        shootRing(3);
        moveArmAuto(true);

        // Turn on the intake and take the ring
        intakeMotor.setPower(0.8);
        movePID(30, 0.4);
        shootRing(1);

        // Align to the power shoot
        movePID(30, 0.6);
        sleep(500);
        intakeMotor.setPower(0);
        sleep(500);
        shooterMotor.setPower(0.725);
        movePIDSide(60, 0.8, true);
        shootPowerShoots(3);
        moveArmAuto(true);

        movePIDSide(100, 0.8, false);
        movePID(200, 0.8);
        moveArmAuto(false);
        sleep(500);
        moveClawAuto(true);
        sleep(1000);
        moveArmAuto(true);
    }

    /*MOVEMENT METHODS*/
    // Move the robot using PID
    public void movePID(double distance, double speed){
        double currentAngle = 0;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid = 0 ;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
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
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower(speed - pid);
            FR.setPower(speed + pid);
            BL.setPower(speed - pid);
            BR.setPower(speed + pid);

            sleep(updateRate);
            lastError = error;

            telemetry.update();
        }

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(angle-currentAngle)>0.7){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower( - pid);
            FR.setPower( + pid);
            BL.setPower( - pid);
            BR.setPower( + pid);

            sleep(updateRate);

            lastError = error;
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Move the robot sideways using PID
    public void movePIDSide(double distance, double speed, boolean isRight){
        double currentAngle;
        final double smoother = 0.2;
        final double threshold = 1;

        // define the PID variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        // This gets the position and makes the robot ready to move
        if (!isRight) speed *=- 1;


        FL.setPower(0.01);
        FR.setPower(0.01);
        BL.setPower(0.01);
        BR.setPower(0.01);


        while ( Math.abs(FL.getCurrentPosition()) < targetEncoder || Math.abs(FR.getCurrentPosition()) < targetEncoder ||
                Math.abs(BL.getCurrentPosition()) < targetEncoder || Math.abs(BR.getCurrentPosition()) < targetEncoder) {

            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            force[0] =   speed - pid;
            force[1] = - speed + pid;
            force[2] = - speed - pid;
            force[3] =   speed + pid;

            // The smoother
            for (int s = 0; s <= 3; s++) {
                if (Math.abs(lastForce[s] - force[s]) > smoother) {
                    if      (lastForce[s] > force[s]) force[s] = lastForce[s] - smoother;
                    else                              force[s] = lastForce[s] + smoother;
                }
            }

            // Save the used force in variables to get the difference
            lastForce[0] = force[0];
            lastForce[1] = force[1];
            lastForce[2] = force[2];
            lastForce[3] = force[3];

            FL.setPower( force[0] );
            FR.setPower( force[1] );
            BL.setPower( force[2] );
            BR.setPower( force[3] );

            telemetry.addData("Moving right", isRight);
            telemetry.update();

            sleep(updateRate);

            lastError = error;
        }

        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (Math.abs(angle-currentAngle)>threshold){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower(- pid);
            FR.setPower(+ pid);
            BL.setPower(- pid);
            BR.setPower(+ pid);

            telemetry.addData("Angle correction", error);
            telemetry.update();

            sleep(updateRate);

            lastError = error;
        }

        // Reset the smoother
        lastForce[0] = 0;
        lastForce[1] = 0;
        lastForce[2] = 0;
        lastForce[3] = 0;
        force[0] = 0;
        force[1] = 0;
        force[2] = 0;
        force[3] = 0;

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stop the motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        telemetry.clear();
        telemetry.update();

    }

    // Program used to precisely turn the robot
    public void turn(double pidK, boolean isRight, double targetAngle) throws InterruptedException {

        final double threshold = 1.1;

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

                FL.setPower( + pid * pidK);
                FR.setPower( - pid * pidK);
                BL.setPower( + pid * pidK);
                BR.setPower( - pid * pidK);

                sleep(updateRate);

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

                FL.setPower(+ pid * pidK);
                FR.setPower(- pid * pidK);
                BL.setPower(+ pid * pidK);
                BR.setPower(- pid * pidK);

                sleep(updateRate);

                lastError=error;
            }
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Move the robot's arm
    public void moveArmAuto(boolean isUp ) {

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
    private void shootPowerShoots(int quantity) throws InterruptedException {

        moveArmAuto(false);
        sleep(500);

        for (int n = 1; n <= quantity; n++) {
            trigServo.setPosition(0.8);
            sleep(1200);
            trigServo.setPosition(0);
            sleep(1000);
            turn(1.35, true, 4);
        }
        shooterMotor.setPower(0);
        moveArmAuto(false);
    }

    // Move the robot's claw
    public void moveClawAuto(boolean isOpen) {
        if(isOpen) {
            clawServo.setPosition(0);
        }
        if(!isOpen) {
            clawServo.setPosition(1);
        }
    }

    // Shoot rings
    private void shootRing(int quantity) throws InterruptedException {

        moveArmAuto(false);
        sleep(500);

        for (int n = 1; n <= quantity; n++) {
            trigServo.setPosition(0.8);
            sleep(1500);
            trigServo.setPosition(0);
            sleep(500);
        }
        shooterMotor.setPower(0);
        moveArmAuto(true);
    }

    /* INITIALIZER METHODS*/
    // Initialize the Vuforia Library
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the IMU
    public void initIMU (){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
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

    // Calibrate the arm
    public void calibrateArm(){

        final int difference = -1100;

        while (!touchSensor.isPressed()) {
            armMotor.setPower(0.3);
            telemetry.addData("Touch", touchSensor.isPressed());
            telemetry.update();
        }

        up = armMotor.getCurrentPosition();
        down = up + difference;
    }
}