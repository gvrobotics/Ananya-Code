package org.firstinspires.ftc.teamcode.NotUsing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Dec14Tele extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR, Slides;
    private CRServo Arm;
    private Servo Claw;
    private boolean gamepad = true;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Slides = hardwareMap.get(DcMotor.class, "Slides");
        Arm = hardwareMap.get(CRServo.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(CRServo.Direction.REVERSE);
        Claw.setDirection(Servo.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Slides.setPower(0);
        Arm.setPower(0);
        Claw.setPosition(0.28);
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            gamepad = !gamepad;
        }

        if (gamepad) {
            powerLX = gamepad1.left_stick_x;
            powerLY = gamepad1.left_stick_y;
            powerRX = gamepad1.right_stick_x;
            powerRY = gamepad1.right_stick_y;

            if (gamepad1.right_bumper || gamepad1.left_bumper)
            {
                powerLX = gamepad1.left_stick_x / 2;
                powerLY = gamepad1.left_stick_y / 2;
                powerRX = gamepad1.right_stick_x / 2;
                powerRY = gamepad1.right_stick_y / 2;
            }

            //HOLONOMIC
            robotAngle = Math.atan2(powerLX, powerLY);
            PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

            lf = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) - powerRX;
            rb = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) + powerRX;
            lb = (PowerMultiplier * Math.sin(robotAngle + (Math.PI / 4))) - powerRX;
            rf = (PowerMultiplier * Math.sin(robotAngle + (Math.PI / 4))) + powerRX;

            FR.setPower(rf);
            FL.setPower(lf);
            BR.setPower(rb);
            BL.setPower(lb);
        } else if (!gamepad) {
            powerLX = gamepad2.left_stick_x;
            powerLY = gamepad2.left_stick_y;
            powerRX = gamepad2.right_stick_x;
            powerRY = gamepad2.right_stick_y;

            if (gamepad2.dpad_right || gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0)
            {
                powerLX = gamepad2.left_stick_x / 2;
                powerLY = gamepad2.left_stick_y / 2;
                powerRX = gamepad2.right_stick_x / 2;
                powerRY = gamepad2.right_stick_y / 2;
            }

            //HOLONOMIC
            robotAngle = Math.atan2(powerLX, powerLY);
            PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

            lf = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) - powerRX;
            rb = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) + powerRX;
            lb = (PowerMultiplier * Math.sin(robotAngle + (Math.PI / 4))) - powerRX;
            rf = (PowerMultiplier * Math.sin(robotAngle + (Math.PI / 4))) + powerRX;

            FR.setPower(rf);
            FL.setPower(lf);
            BR.setPower(rb);
            BL.setPower(lb);
        }

        //INTAKE/OUTAKE
        //SLIDES - dpad
        //up
        if (gamepad2.dpad_up) {
            Slides.setPower(0.8);
        } else {
            Slides.setPower(0.2);
        }
        //down
        if (gamepad2.dpad_down) {
            Slides.setPower(-0.3);
        } else {
            Slides.setPower(0);
        }
        //off
        if(gamepad2.dpad_left) {
            Slides.setPower(0);
        }

        //ARM - bumpers
        //up
        if (gamepad2.right_bumper) {
            Arm.setPower(0.9);
        }
        //down
        if (gamepad2.left_bumper) {
            Arm.setPower(-0.7);
        }
        //off
        if (gamepad2.a) {
            Arm.setPower(0);
        }

        //CLAW - buttons
        //open
        if (gamepad2.b) {
            Claw.setPosition(0.15);
        }
        //close
        if (gamepad2.x) {
            Claw.setPosition(0.28);
        }

        // Telemetry
        telemetry.addData("powerRX: ", gamepad ? gamepad1.right_stick_x : gamepad2.right_stick_x);
        telemetry.addData("powerRY: ", gamepad ? gamepad1.right_stick_y : gamepad2.right_stick_y);
        telemetry.addData("powerLX: ", gamepad ? gamepad1.left_stick_x : gamepad2.left_stick_x);
        telemetry.addData("powerLY: ", gamepad ? gamepad1.left_stick_y : gamepad2.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());

        telemetry.addData("Slides: ", Slides.getPower());
        telemetry.addData("Arm: ", Arm.getPower());
        telemetry.addData("Claw: ", Claw.getPosition());

        telemetry.addData("Gamepad: ", gamepad ? "gamepad1" : "gamepad2");

        telemetry.update();
    }
}
