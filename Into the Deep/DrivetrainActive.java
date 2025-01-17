package org.firstinspires.ftc.teamcode.SlidesTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Drivetrain Active")
public class DrivetrainActive extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR, VSlideR1, VSlideL2;
    private boolean drivetrainActive = true;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        VSlideR1 = hardwareMap.get(DcMotor.class, "VSR");
        VSlideL2 = hardwareMap.get(DcMotor.class, "VSL");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        VSlideR1.setDirection(DcMotorSimple.Direction.FORWARD);
        VSlideL2.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSlideR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSlideL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        VSlideR1.setPower(0);
        VSlideL2.setPower(0);
    }

    @Override
    public void loop() {
        if (gamepad1.y || gamepad2.y) {
            drivetrainActive = !drivetrainActive;
        }

        if (drivetrainActive) {
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

            BR.setPower(rb);
            BL.setPower(lb);
            FR.setPower(rf);
            FL.setPower(lf);

        } else if (!drivetrainActive) {
            // Shut off drivetrain motors
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }

        //INTAKE/OUTTAKE
        //Slides - dpad
        //up
        if (gamepad1.dpad_up) {
            VSlideR1.setPower(1.0);
            VSlideL2.setPower(1.0);
        } else {
            VSlideR1.setPower(0);
            VSlideL2.setPower(0);
        }
        //down
        if (gamepad1.dpad_down) {
            VSlideR1.setPower(-0.8);
            VSlideL2.setPower(-0.8);
        } else {
            VSlideR1.setPower(0);
            VSlideL2.setPower(0);
        }

        //OTHER
        //up
        if (gamepad2.dpad_up) {
            VSlideL2.setPower(1.0);
        } else {
            VSlideL2.setPower(0);
        }
        if (gamepad2.dpad_right) {
            VSlideR1.setPower(1.0);
        } else {
            VSlideR1.setPower(0);
        }
        //down
        if (gamepad2.dpad_down) {
            VSlideL2.setPower(-0.8);
        } else {
            VSlideL2.setPower(0);
        }
        if (gamepad2.dpad_left) {
            VSlideR1.setPower(-0.8);
        } else {
            VSlideR1.setPower(0);
        }

        // Telemetry
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());

        telemetry.addData("VSlide1: ", VSlideR1.getPower());
        telemetry.addData("VSlide2: ", VSlideL2.getPower());

        telemetry.addData("Drivetrain Active: ", drivetrainActive);

        telemetry.update();
    }
}
