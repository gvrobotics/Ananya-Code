package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SingleTele extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR, Slides;
    private Servo Arm, Claw;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Slides = hardwareMap.get(DcMotor.class, "Slides");
        Arm = hardwareMap.get(Servo.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(Servo.Direction.REVERSE);
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
        Arm.setPosition(0);
        Claw.setPosition(0);
    }

    @Override
    public void loop() {
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


        //INTAKE/OUTAKE
        //SLIDES - dpad
        //up
        if (gamepad1.dpad_up) {
            Slides.setPower(0.6);
        } else {
            Slides.setPower(0);
        }
        //down
        if (gamepad1.dpad_down) {
            Slides.setPower(-0.3);
        } else {
            Slides.setPower(0);
        }

        //ARM - bumpers
        if(gamepad1.right_bumper) {
            Arm.setPosition(0.5);
        }
        if (gamepad1.left_bumper) {
            Arm.setPosition(0);
        }

        //CLAW - buttons
        //Open
        if (gamepad1.b) {
            Claw.setPosition(0.15);
        }
        //Close
        if (gamepad1.x) {
            Claw.setPosition(0.25);
        }

        // telemetry
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());

        telemetry.addData("Slides: ", Slides.getPower());
        telemetry.addData("Arm: ", Arm.getPosition());
        telemetry.addData("Claw: ", Claw.getPosition());

        telemetry.update();
    }
}