package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Other Tank", group = "Tank")
public class TankDriveButtons extends OpMode
{
    public DcMotor BR, BL, FR, FL;

    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        // Adjust these based on your robot’s wiring
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Button Drive Initialized ✅");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        double power = 0.5; // adjust speed here

        // --- Move Forward (A Button) ---
        if (gamepad1.x) {
            FL.setPower(power);
            FR.setPower(power);
            BL.setPower(power);
            BR.setPower(power);
        }

        // --- Move Backward (B Button) ---
        if (gamepad1.b) {
            FL.setPower(-power);
            FR.setPower(-power);
            BL.setPower(-power);
            BR.setPower(-power);
        }

        // --- Strafe Right (X Button) ---
        if (gamepad1.a) {
            FL.setPower(power);
            FR.setPower(-power);
            BL.setPower(-power);
            BR.setPower(power);
        }

        // --- Strafe Left (Y Button) ---
        if (gamepad1.y) {
            FL.setPower(-power);
            FR.setPower(power);
            BL.setPower(power);
            BR.setPower(-power);
        }

        // --- Rotate Right (Right Bumper) ---
        if (gamepad1.right_bumper) {
            FL.setPower(power);
            FR.setPower(-power);
            BL.setPower(power);
            BR.setPower(-power);
        }

        // --- Rotate Left (Left Bumper) ---
        if (gamepad1.left_bumper) {
            FL.setPower(-power);
            FR.setPower(power);
            BL.setPower(-power);
            BR.setPower(power);
        }

        // --- Stop if no button pressed ---
        if (gamepad1.start) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

        telemetry.addLine("Press A=Forward, B=Back, X=Right, Y=Left, LB/RB=Rotate");
        telemetry.update();
    }
}
