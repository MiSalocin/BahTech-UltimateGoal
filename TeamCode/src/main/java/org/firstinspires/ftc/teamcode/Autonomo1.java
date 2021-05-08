/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@Disabled
@Autonomous(name = "Autonomo1", group = "Run1", preselectTeleOp = "TeleOp")
public class Autonomo1 extends LinearOpMode
{
    DcMotor			 frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Coletor;
    Servo            Garra;
    BNO055IMU		 imu;
    Orientation		 lastAngles = new Orientation();
    double		   	 globalAngle = .30;

    double			  DIAMETRO_IN = 3.966;
    double            POLEGADA = 2.54;
    double			  ENGRENAGEM = 2.6;
    int			      TICKS_REV = 1120;

    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;

    Recognition recognition;

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Inicializando");
        telemetry.update();

        List<Recognition> recognitions;
        double index;

        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        // This sample assumes phone is in landscape mode.
        // Rotate phone -90 so back camera faces "forward" direction on robot.
        // We need Vuforia to provide TFOD with camera images.
        vuforiaUltimateGoal.initialize(
                "BOTAR A LICENÇA AQUI", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                -90, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        tfodUltimateGoal.setZoom(1.5, 16 / 9);


        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        backLeftMotor = hardwareMap.dcMotor.get("back_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        backRightMotor = hardwareMap.dcMotor.get("back_right");
        Coletor = hardwareMap.get(DcMotor.class, "coletor");

        Garra = hardwareMap.get(Servo.class, "garra");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        Coletor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData(">", "Bora dale");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfodUltimateGoal.getRecognitions();
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (recognitions.size() == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Zona Alvo", "A");
                    doZonaA();
                    break;
                } else
                    {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                    if (recognition.getLabel() == "Single") {
                        telemetry.addData("Zona Alvo", "B");
                        doZonaB();
                        break;
                    } else if (recognition.getLabel() == "Quad") {
                        telemetry.addData("Zona Alvo", "C");
                        doZonaC();
                        break;
                    }
                }
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfodUltimateGoal.deactivate();

        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();
    }

    private void displayInfo(double i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, recognition.getLeft() + ", " + recognition.getTop());
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, recognition.getRight() + ", " + recognition.getBottom());
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        degrees -= 15; //erro de 15º

        // reseta o ângulo do imu
        resetAngle();

        int i = 0;
        do {
            if (degrees < 0) {   // gira para direita
                leftPower = power;
                rightPower = -power;
            } else if (degrees > 0) {   // gira para esquerda
                leftPower = -power;
                rightPower = power;
            } else return;

            // define a potência para o giro
            frontLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backLeftMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
            i++;

            while(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
                telemetry.addData("Situação", "Girando");
                telemetry.addData("Posição/Rotação", frontRightMotor.getCurrentPosition());
                telemetry.addData("Angulo", getAngle());
                telemetry.update();
            }

        }while(i != 2);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else	// left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // desliga os motores
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // aguarda a rotação para parar
        sleep(500);

        // reseta o ângulo do imu
        resetAngle();
    }

    private void toFrente(int distancia, double power)
    {
        double CIRCUNFERENCIA = Math.PI * DIAMETRO_IN;
        double DISTANCIA_IN = distancia/POLEGADA;
        double ROTACOES = (DISTANCIA_IN / CIRCUNFERENCIA ) / ENGRENAGEM;
        int targetEncoder = (int)(ROTACOES*TICKS_REV);

        resetEncoder();

        frontRightMotor.setTargetPosition(targetEncoder);
        backRightMotor.setTargetPosition(targetEncoder);
        frontLeftMotor.setTargetPosition(targetEncoder);
        backLeftMotor.setTargetPosition(targetEncoder);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);

        while(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
            telemetry.addData("Situação", "Andando");
            telemetry.addData("Posição/Rotação", frontRightMotor.getCurrentPosition());
            telemetry.addData("Angulo", getAngle());
            telemetry.update();
        }

        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void resetEncoder()
    {
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void doZonaA()
    {

    }
    private void doZonaB()
    {

    }
    private void doZonaC()
    {

    }
}
