package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Tele_AN extends OpMode
{
    private double powerLX, powerLY, powerRX, powerRY;
    private DcMotor FL, BL, FR, BR, Arm0, Arm1;
    private Servo Launcher;

    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Arm0 = hardwareMap.get(DcMotor.class, "Arm0");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Launcher = hardwareMap.get(Servo.class, "Launcher");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.FORWARD);


        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Arm0.setPower(0);
        Arm1.setPower(0);
        Launcher.setPosition(0);

    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        if(powerLY > 0.05 || powerLY < -0.05)
        {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        } else
        {
            FL.setPower(0);
            BL.setPower(0);
        }

        if(powerRY > 0.05 || powerRY < -0.05)
        {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        } else
        {
            FR.setPower(0);
            BR.setPower(0);
        }

        if(gamepad1.right_bumper){
            FL.setPower(-0.85);
            BL.setPower(0.85);
            FR.setPower(0.85);
            BR.setPower(-0.85);
        }
        if(gamepad1.left_bumper){
            FL.setPower(0.85);
            BL.setPower(-0.85);
            FR.setPower(-0.85);
            BR.setPower(0.85);
        }

        //intake/outake

        //going up
        if (gamepad2.dpad_up)
        {
            Arm0.setPower(0.5);
            Arm1.setPower(0.5);
        }
        //stopping when going up
        if(gamepad2.dpad_right) {
            Arm0.setPower(-0.15);
            Arm1.setPower(-0.15);
        }
        //going down
        if(gamepad2.dpad_down){
            Arm0.setPower(-0.2);
            Arm1.setPower(-0.2);
        }
        //stop when going down
        if(gamepad2.dpad_left){
            Arm0.setPower(0.07);
            Arm1.setPower(0.07);
        }
        //stop TOTALLY / reset
        if(gamepad2.x){
            Arm0.setPower(0);
            Arm1.setPower(0);
        }
        //to make it sit in the air
        if(gamepad2.right_bumper){
            Arm0.setPower(0.35);
            Arm1.setPower(0.35);
        }


        //to shoot launcher
        if(gamepad2.a){
            Launcher.setPosition(0.4);
        }

        //reset launcher
        if(gamepad2.y){
            Launcher.setPosition(0);
        }

        // telemetry
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);


        telemetry.addData("FR START: ", FR.getCurrentPosition());
        telemetry.addData("FL START: ", FL.getCurrentPosition());
        telemetry.addData("BR START: ", BR.getCurrentPosition());
        telemetry.addData("BL START: ", BL.getCurrentPosition());

        telemetry.addData("Arm0: ", Arm0.getPower());
        telemetry.addData("Arm0: ", Arm1.getPower());
        telemetry.addData("Launcher: ", Launcher.getPosition());
        telemetry.update();
    }
}