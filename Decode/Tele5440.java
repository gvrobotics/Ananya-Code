package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TeleOp 5440", group = "Tele")
public class Tele5440 extends OpMode {
    public DcMotor BR, BL, FR, FL, spin1, spin2;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private Servo flip;
    private ElapsedTime timer = new ElapsedTime();
    private Boolean motorOn = false, t = false, previousGamepad = false, currentGamepad = false;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        spin1 = hardwareMap.get(DcMotor.class, "s1");
        spin2 = hardwareMap.get(DcMotor.class, "s2");
        flip = hardwareMap.get(Servo.class, "f");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        spin1.setDirection(DcMotorSimple.Direction.FORWARD);
        spin2.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        spin1.setPower(0);
        spin2.setPower(0);
        flip.setPosition(0);
    }

    @Override
    public void loop() {
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;

        robotAngle = Math.atan2(powerLX, powerLY);
        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier * (Math.sin(robotAngle + (Math.PI / 4)))) - powerRX;
        rb = (PowerMultiplier * (Math.sin(robotAngle + (Math.PI / 4)))) + powerRX;
        lb = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) - powerRX;
        rf = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) + powerRX;

        BR.setPower(rb);
        BL.setPower(lb);
        FR.setPower(rf);
        FL.setPower(lf);

        // OUTTAKE
        // flip
        if (gamepad1.right_bumper && !t) {
            // When right bumper is pressed, flip down
            flip.setPosition(0.2);
            timer.reset();
            t = true;
        } else if (t && !gamepad1.right_bumper && timer.seconds() > 0.3) {
            // After right bumper is released for 0.3s, flip back up
            flip.setPosition(0);
            t = false;
        }

        // TOGGLE FOR SHOOTER

        // Stores the previous state of the button
        previousGamepad = currentGamepad;
        // Read the current state of bumper
        currentGamepad = gamepad1.left_bumper;

        // Checks if the button was just pressed
        if (currentGamepad && !previousGamepad) {
            motorOn = !motorOn;
            if (motorOn) {
                spin1.setPower(0.45);
                spin2.setPower(0.45);
            } else {
                spin1.setPower(0);
                spin2.setPower(0);
            }
        }

        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("spin1: ", spin1.getPower());
        telemetry.addData("spin2: ", spin2.getPower());
        telemetry.addData("flip: ", flip.getPosition());
        telemetry.addData("Outtake On: ", motorOn);
        telemetry.update();
    }
}
