package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "MMCoreOP", group = "Bahtech")
public class MMCore extends OpMode {

    // Initialize variables to object detection
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Map the hardware to autonomus and teleOp
    final MMMovementTO moveTele = new MMMovementTO();
    final MMMovementAU moveAuto = new MMMovementAU();

    double x = 1;
    int n = 0;

    @Override
    public void init() {

        // Create a Tensorflow list and star vuforia and TF
        List<Recognition> rec = tfod.getUpdatedRecognitions();
        initVuforia(); initTfod(); tfod.activate();
        tfod.setZoom(2.5, 16.0/9.0);

        // Create the Hardware Map in both our classes
        moveAuto.defHardware(hardwareMap);
        moveTele.defHardware(hardwareMap);
        if(tfod != null){
            if (rec != null) {
                telemetry.addData("Object Detected: ", "");
                for(Recognition show : rec){
                    telemetry.addData("There is: ",show.getLabel());
                    telemetry.addData("  Left, Top (%d)", "%.03f , %.03f",
                            show.getLeft(), show.getTop());
                    telemetry.addData("  Right, Bottom: ", "%.03f , %.03f",
                            show.getRight(), show.getBottom());
                }
            }
        }
        telemetry.addData("Press play to start TeleOp Mode", "");
}

    @Override
    public void loop() {

        // Show the buttons on the screen
        telemetry.addData("Left  Stick  X: ", gamepad1.left_stick_x);
        telemetry.addData("Left  Stick  Y: ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick  X: ", gamepad1.right_stick_x);
        telemetry.addData("Force: ", x);
        telemetry.update();

        if (gamepad1.x)
            x = 0.75;
        if (gamepad1.y)
            x = 0.8;

        // Create variables from the control
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        n++; telemetry.addData("PCA", n);

        moveTele.moveRobot(leftX, leftY, rightX, lb, rb);
    }

    // Configure and initialize Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Ad0n+XX/////AAABmQ/41s5hKkoQl9XvVGzFatosnvXWi3lcaK406bSM6BCi" +
                "UoPYCNo83nOVmmi0PcL1v6+gDcdnZddtNmRaYSKGqMhsoHzczhJbbzJa6vsQPZ6Bzzs/9icQySfGBy4wUXN" +
                "FBysun4H4G2qQEeaWP/PwNu7FkREo28S5DXYbk5G1SToOk/6MiOG4v8zuy1WvA2Mrg2gbJMlj181zI7Wj+C" +
                "YJXDUpE2ugflgq9iDDzBCJGb0r1/sEwkqpBq/u3G8F2iPK5kRxLSsz2yB8AjpiLZlQJoZ9NpIwkEyuS9i6A" +
                "Vc9lnPTMst+gortmeJ+ThBBhscsJ55tdDf8yLn11cQgJSyrWBr9+9HIyvRc3ixR/og6y/Mc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the TensorFlow Object Detection engine
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("UltimateGoal.tflite", "Four", "One");
    }
}