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

    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    //Hardware variables
    ColorSensor colorSensor;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor armMotor;
    DcMotor intake;
    DcMotor shooterMotor;
    Servo clawServo;
    Servo trigServo;
    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    //Encoder constants
    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 1.25;
    final double WHEEL_DIAMETER_CM = 10.16;

    // defines the constant multipliers to the PID
    final double kp = 1;
    final double ki = 0.4;
    final double kd = 1.5;
    final double k = 50;
    double i = 0;
    double angle = 0;

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
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        trigServo = hardwareMap.get(Servo.class, "trig_servo");

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


//        moveClawAuto(false);
//        sleep(1200);
//        moveArmAuto(true);

        //Initialize Vuforia and TensorFlow
        initVuforia();
        initTfod();

        //Show a message to press play
        moveArmAuto(true);
        sleep(1000);
        moveClawAuto(false);

        telemetry.addData(">", "Press Play to start!");
        telemetry.update();

        waitForStart();

        if (tfod != null) {
            tfod.activate();
        } //Activate TensorFlow

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                sleep(2500);
                recognitions = tfod.getRecognitions();
                tfod.shutdown();
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

    private void goWhite() {
        double speed = 0.7;
        int whiteTape = 500; //Alpha
        int redTape = 110; //Based on RGB
;
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

    private void goZoneA(){

        //Delivery 1st wobble goal
        goWhite();
        moveArmAuto(false);
        sleep(1200);
        moveClawAuto(true);
        sleep(100);
        moveArmAuto(true);
        sleep(500);
        moveClawAuto(false);
        sleep(500);

        //Go and align to the power shoot position
        shooterMotor.setPower(0.68);
        movePD(-45, 0.7);
        movePDSide(70, 0.7, true);
        sleep(500);
        movePDSide(70, 0.7, true);
        sleep(500);
        shootPowerShoots(3);
        turn(0.5,false,11,50);
        goWhite();
        sleep(5000);

    }

    private void goZoneB(){
        tfod.shutdown();

        //Align to the starter stack by strafing right
        movePDSide(45, 0.6, true);
        sleep(500);

        //Get near of the stack
        movePD(35, 0.4);
        sleep(1500);
        shootRing(2, 0.7);
        sleep(1000);

        //Turn on the intake and take the ring
        intake.setPower(1);
        movePD(40, 0.3);

        //Align to the power shoot
        intake.setPower(0);
        sleep(500);
        movePDSide(80, 0.6, true);
        sleep(1000);
        goWhite();

//        movePD(55, 0.6);
//        sleep(500);
//        moveArmAuto(false);
//        sleep(1000);
//        moveClawAuto(true);
//        sleep(1000);
//        moveArmAuto(true);
//        sleep(1000);
//        moveClawAuto(false);
//        movePD(-185, 0.7);
//        sleep(1000);
//        moveClawAuto(true);
//        moveArmAuto(false);
//        sleep(5000);


        /*
        movePD(30,0.5);
        sleep(1500);

        //Shoot three powerShoots
        turn(0.6, true, 3, 30 );
        sleep(1500);
        turn(0.6, true, 3, 30 );
        sleep(1500);
        turn(0.6, false, 12, 45);
        sleep(1500);

        //Delivery the 1st wobble goal
        goWhite();
        turn(0.6, true, 10, 40);
        movePD(30, 0.6);
        moveArmAuto(false);
        sleep(1000);
        moveClawAuto(true);
        sleep(1000);
        moveArmAuto(true);
        sleep(1000);
        moveClawAuto(false);
        sleep(5000);
        */
    }

    private void goZoneC(){
        tfod.shutdown();
        goWhite();
        movePD(75, 0.4);
        moveArmAuto(false);
        sleep(1300);
        moveClawAuto(true);
        sleep(1600);
        moveArmAuto(true);
        moveClawAuto(false);

        //Go shoot powershoots

    }

    /*MOVEMENT METHODS*/
    // Move the robot using PD
    public void movePD(double distance, double speed){
        double currentAngle = 0;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid = 0 ;
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
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            telemetry.addData("PID VALUE", pid);

            FL.setPower(speed - pid);
            FR.setPower(speed + pid);
            BL.setPower(speed - pid);
            BR.setPower(speed + pid);

            telemetry.addData("PID", FL.getPower());
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);

            sleep(100);
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

            FL.setPower(- pid);
            FR.setPower(+ pid);
            BL.setPower(- pid);
            BR.setPower(+ pid);

            telemetry.addData("PID", pid);
            telemetry.addData("P",p);
            telemetry.addData("I",i);
            telemetry.addData("D",d);
            telemetry.update();

            sleep(100);

            lastError = error;
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void movePDSide(double distance, double speed, boolean isRight){
        double currentAngle;
        final double smoother = 0.03;
        final double threshold = 1;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double circumference = Math.PI * WHEEL_DIAMETER_CM;
        double rotation = (distance / circumference) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        if (isRight) {
            FL.setTargetPosition(targetEncoder);
            FR.setTargetPosition(-targetEncoder);
            BR.setTargetPosition(targetEncoder);
            BL.setTargetPosition(-targetEncoder);
        }else {
            FL.setTargetPosition(-targetEncoder);
            FR.setTargetPosition(targetEncoder);
            BR.setTargetPosition(-targetEncoder);
            BL.setTargetPosition(targetEncoder);
        }
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(0.01);
        FR.setPower(0.01);
        BL.setPower(0.01);
        BR.setPower(0.01);


        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {

            force[0] = speed;
            force[1] = speed;
            force[2] = speed;
            force[3] = speed;

            if (Math.abs(lastForce[0] - force[0]) > smoother) {
                if (lastForce[0] > force[0]) force[0] = lastForce[0] - smoother;
                else force[0] = lastForce[0] + smoother;
            }
            if (Math.abs(lastForce[1] - force[1]) > smoother) {
                if (lastForce[1] > force[1]) force[1] = lastForce[1] - smoother;
                else force[1] = lastForce[1] + smoother;
            }
            if (Math.abs(lastForce[2] - force[2]) > smoother) {
                if (lastForce[2] > force[2]) force[2] = lastForce[2] - smoother;
                else force[2] = lastForce[2] + smoother;
            }
            if (Math.abs(lastForce[3] - force[3]) > smoother) {
                if (lastForce[3] > force[3]) force[3] = lastForce[3] - smoother;
                else force[3] = lastForce[3] + smoother;
            }

            // Save the used force in variables to get the difference
            lastForce[0] = force[0];
            lastForce[1] = force[1];
            lastForce[2] = force[2];
            lastForce[3] = force[3];

            FL.setPower(force[0]);
            FR.setPower(force[1]);
            BL.setPower(force[2]);
            BR.setPower(force[3]);

            telemetry.addData("Moving to target", "");
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
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

            telemetry.addData("PID", pid);
            telemetry.addData("P",p);
            telemetry.addData("I",i);
            telemetry.addData("D",d);
            telemetry.addData("",currentAngle + threshold < angle || currentAngle - threshold > angle);
            telemetry.update();

            sleep(100);

            lastError = error;
        }

        // the PID in action
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        telemetry.clear();
        telemetry.update();

    }

    // Program used to precisely turn the robot
    public void turn(double force, boolean right, double targetAngle, final double smoother) {
        final int threshold = 10;
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
                telemetry.addData("angulo", angle);
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
                telemetry.addData("angulo", angle);
            }
        }
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    // Move the robot's arm
    public void moveArmAuto(boolean isUp ) {

        final int up = 670;
        final int down = -460;
        final double force = 0.9;



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
    public void moveClawAuto(boolean isOpen) {
        if(isOpen) {
            clawServo.setPosition(0);
        }
        if(!isOpen) {
            clawServo.setPosition(1);
        }
    }

    private void shootPowerShoots(int quantity) {
        moveArmAuto(false);
        for (int n = 1; n <= quantity; n++) {
            trigServo.setPosition(0.2);
            sleep(1500);
            trigServo.setPosition(1);
            sleep(1500);
            turn(0.5, true, 4, 40);
        }
        shooterMotor.setPower(0);
    }

    private void shootRing(int quantity, double shooterPower) {
        shooterMotor.setPower(shooterPower);

        for (int n = 1 ; n <= quantity; n++) {
            trigServo.setPosition(0.2);
            sleep(500);
            trigServo.setPosition(1);
            sleep(500);
        }
        shooterMotor.setPower(0);
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

    public double pid (double targetAngle, double currentAngle){
        double error;
        double p;
        double d;
        double pid;
        double lastError = 500;

        error = targetAngle - currentAngle;

        p = error * kp;
        i += error;
        d = (error - lastError) * kd;
        pid = (p + d) / k;

        sleep(200);
        lastError = error;
        return pid;
    }

}