package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MovementAU {

    // Variáveis usadas na garra e no tiro
    private int    x = 0;
    private double y = 1;

    // Motores para movimento
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    // Motores usados na garra
    private DcMotor arm;
    private Servo hand;

    //Motores usados no tiro
    private DcMotor shooter;
    private Servo shooT;

    void defHardware (HardwareMap local) {

        // Motores usados no movimento
        frontLeft = local.dcMotor.get("front_left_motor");
        frontRight = local.dcMotor.get("front_right_motor");
        backLeft = local.dcMotor.get("back_left_motor");
        backRight = local.dcMotor.get("back_right_motor");

        // Reverte motores da direita para melhor controle
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Motores usados na garra
        arm = local.dcMotor.get("arm_motor");
        hand = local.servo.get("hand_servo");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motores usados no tiro
        DcMotor intake = local.dcMotor.get("intake_motor");
        shooter = local.dcMotor.get("shooter_motor");
        shooT = local.servo.get("shooter_trig_servo");

        // Motores que permanecerão ligados
        intake.setPower(1);
    }

    void moveRobo (double leftX, double leftY, double rightX, boolean slower, boolean faster){

        //  Se o bumper da esquerda for pressionado, o robô se mexerá em velocidade baixa

        if (slower) {
            frontRight.setPower((+leftY + leftX - rightX) / 4);
            backRight .setPower((+leftY - leftX - rightX) / 4);
            frontLeft .setPower((+leftY - leftX + rightX) / 4);
            backLeft  .setPower((+leftY + leftX + rightX) / 4);
        }

        // Se o bumper da direita for pressionado, o robô se mexerá em velocidade alta
        else if (faster) {
            frontRight.setPower( + leftY + leftX - rightX );
            backRight .setPower( + leftY - leftX - rightX );
            frontLeft .setPower( + leftY - leftX + rightX );
            backLeft  .setPower( + leftY + leftX + rightX );
        }

        // Se nenhum dos botões forem pressionados, o robô se mexerá em velocidade média
        else {
            frontRight.setPower(( + leftY + leftX - rightX ) / 2);
            backRight .setPower(( + leftY - leftX - rightX ) / 2);
            frontLeft .setPower(( + leftY - leftX + rightX ) / 2);
            backLeft  .setPower(( + leftY + leftX + rightX ) / 2);
        }
    }

    void moveGarra(boolean up, boolean down, boolean openHand, boolean closeHand){

        // Levanta ou abaixa a garra
        if (up   && -1   >= arm.getCurrentPosition()) {
            arm.setTargetPosition(x+=5);
            arm.setPower(1);
        }else if (down && -653 <= arm.getCurrentPosition()) {
            arm.setTargetPosition(x-=5);
            arm.setPower(1);
        }else arm.setPower(0);

        // Abre ou fecha a "mão" do robô
        if (openHand  && y < 1) hand.setPosition(y += 0.05);
        if (closeHand && y > 0) hand.setPosition(y -= 0.05);

    }

    void tiro(boolean trigger, double force){

        shooter.setPower(force);
        if (trigger) shooT.setPosition(1);
        else   shooT.setPosition(0);

    }

    public int getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}