package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BlazinAnanya extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY;
    private DcMotor FL, BL, FR, BR;
    public Servo claw;
    public DcMotor linear;

    @Override
    public void init() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(Servo.class, "c");
        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setPower(0);
        claw.setPosition(0);

    }

    @Override
    public void loop() {
        powerLX = gamepad1.left_stick_x / 2;
        powerLY = gamepad1.left_stick_y / 2;
        powerRX = gamepad1.right_stick_x / 2;
        powerRY = gamepad1.right_stick_y / 2;

        if (powerLY > 0.07 || powerLY < -0.07) {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        } else {
            FL.setPower(0);
            BL.setPower(0);
        }

        if (powerRY > 0.07 || powerRY < -0.07) {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        } else {
            FR.setPower(0);
            BR.setPower(0);

            if (gamepad1.dpad_up) {
                linear.setPower(0.5);
            } else {
                linear.setPower(0);

                if (gamepad1.dpad_down) {
                    linear.setPower(-0.5);
                } else {
                    linear.setPower(0);

                }

                if (gamepad1.a) {
                    claw.setPosition(0.45);
                } else if (gamepad1.y) {
                    claw.setPosition(0);

                }
            }
        }
        if (gamepad1.x) {
            BR.setPower(-0.5);
            BL.setPower(0.5);
            FR.setPower(0.5);
            FL.setPower(-0.5);
        }
        if (gamepad1.b) {
            BR.setPower(0.5);
            BL.setPower(-0.5);
            FR.setPower(-0.5);
            FL.setPower(0.5);
        }
    }
}
