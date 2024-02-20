package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Tele_3795 extends OpMode
{
    private double powerLX, powerLY, powerRX, powerRY;
    private DcMotor FL, BL, FR, BR, Arm0, Arm1;
    private Servo Launcher, Wrist, Claw;

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
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");


        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
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
        Wrist.setPosition(0.2);
        Claw.setPosition(0);
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

        //STRAFING
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

        //ARM
        //going up
        if (gamepad2.dpad_up) {
            Arm0.setPower(0.63);
            Arm1.setPower(0.63);
        }
        //going down
        if(gamepad2.dpad_down){
            Arm0.setPower(-0.2);
            Arm1.setPower(-0.2);
        }
        //stopping when going up
        if(gamepad2.dpad_left) {
            Arm0.setPower(0.1);
            Arm1.setPower(0.1);
        }
        //to move it a little off the ground
        if(gamepad2.right_stick_button){
            Arm0.setPower(0.4);
            Arm1.setPower(0.4);
        }
        //RIGGING
        //max power down
        if(gamepad2.dpad_right) {
            Arm0.setPower(-1);
            Arm1.setPower(-1);
        }
        //stop TOTALLY / reset
        if(gamepad2.x){
            Arm0.setPower(0);
            Arm1.setPower(0);
        }
        /*
        //stop when going down
        if(gamepad2.dpad_left){
            Arm0.setPower(0.09);
            Arm1.setPower(0.09);
        } */

        //WRIST
        if(gamepad2.left_bumper){
            Wrist.setPosition(0.2);
        }
        if(gamepad2.right_bumper){
            Wrist.setPosition(0.95);
        }

        //CLAW
        //open claw wide
        if(gamepad2.y){
            Claw.setPosition(0.8);
        }
        //open claw slightly
        if(gamepad2.b){
            Claw.setPosition(0.65);
        }
        //close claw
        if(gamepad2.a){
            Claw.setPosition(0);
        }

        //LAUNCHER
        if(gamepad1.a){
            Launcher.setPosition(0.4);
        }
        //reset launcher
        if(gamepad1.y){
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
        telemetry.addData("Wrist: ", Wrist.getPosition());
        telemetry.addData("Launcher: ", Launcher.getPosition());
        telemetry.addData("Claw: ", Claw.getPosition());
        telemetry.addData("Wrist: ", Wrist.getPosition());

        telemetry.update();
    }
}