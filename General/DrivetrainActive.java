package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Drivetrain Active")
public class DrivetrainActive extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR;
    private boolean drivetrainActive = true;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
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

        // Telemetry
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());

        telemetry.addData("Drivetrain Active: ", drivetrainActive);

        telemetry.update();
    }
}
