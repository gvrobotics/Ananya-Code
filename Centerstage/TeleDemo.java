package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleDemo extends OpMode
{
    private double powerLX, powerLY, powerRX, powerRY;
    private DcMotor FL, BL, FR, BR, Rigging1, Rigging2;
    private Servo Wrist1, Wrist2;

    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Rigging1 = hardwareMap.get(DcMotor.class, "Rigging1");
        Rigging2 = hardwareMap.get(DcMotor.class, "Rigging2");

        Wrist1 = hardwareMap.get(Servo.class, "Wrist2");
        Wrist2 = hardwareMap.get(Servo.class, "Wrist2");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Rigging1.setDirection(DcMotorSimple.Direction.FORWARD);
        Rigging2.setDirection(DcMotorSimple.Direction.REVERSE);

        Wrist1.setDirection(Servo.Direction.FORWARD);
        Wrist2.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Rigging1.setPower(0);
        Rigging2.setPower(0);

        Wrist1.setPosition(0);
        Wrist2.setPosition(0);
    }


    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        //TANK DRIVE
        // CHANGED WHEELS TO NEGATIVE POWER
        if(powerLY > 0.05 || powerLY < -0.05)
        {
            FL.setPower(-powerLY);
            BL.setPower(-powerLY);
        } else
        {
            FL.setPower(0);
            BL.setPower(0);
        }

        if(powerRY > 0.05 || powerRY < -0.05)
        {
            FR.setPower(-powerRY);
            BR.setPower(-powerRY);
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
        //INTAKE/OUTAKE

        //5440 RIGGING
        // 4.4 inches in 1 rotation

        if (gamepad1.dpad_up)
        {
            Rigging1.setPower(0.325);
            Rigging2.setPower(0.3);
        }
        if (gamepad1.dpad_down)
        {
            Rigging1.setPower(-0.325);
            Rigging2.setPower(-0.3);
        }
        if(gamepad1.dpad_left){
            Rigging1.setPower(0);
            Rigging2.setPower(0);
        }

        //WRIST
        if(gamepad1.a){
            Wrist1.setPosition(0.5);
            Wrist2.setPosition(0.5);
        }
        if(gamepad1.y){
            Wrist1.setPosition(0.9);
            Wrist2.setPosition(0.9);
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

        telemetry.addData("Wrist1: ", Wrist1.getPosition());
        telemetry.addData("Wrist2: ", Wrist1.getPosition());

        telemetry.addData("Rigging1: ", Rigging1.getPower());
        telemetry.addData("Rigging2", Rigging2.getPower());
        telemetry.update();
    }

}