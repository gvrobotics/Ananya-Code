package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Tele3795 extends OpMode {
    public DcMotor BR, BL, FR, FL;
    public DcMotorEx intake, fly1, fly2, transfer;
    private Boolean prevRB1 = false, currRB1 = false, shooterOn = false;
    private Boolean prevLB = false, currLB = false, intakeOn = false;
    private Boolean prevA = false, currA = false, transOn = false;

    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake = hardwareMap.get(DcMotorEx.class, "i1");
        transfer = hardwareMap.get(DcMotorEx.class, "i1");

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fly1.setPower(0);
        fly2.setPower(0);
        intake.setPower(0);
        transfer.setVelocity(0);
    }

    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double BRp = (y + x - rx);
        double BLp = (y - x + rx);
        double FRp = (y - x - rx);
        double FLp = (y + x + rx);

        BR.setPower(BRp);
        BL.setPower(BLp);
        FR.setPower(FRp);
        FL.setPower(FLp);

        // ============= SHOOTER TOGGLE (Right Bumper) ===========
        prevRB1 = currRB1;
        currRB1 = gamepad1.right_bumper;
        if (currRB1 && !prevRB1) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                fly1.setPower(0.5);
                fly2.setPower(0.5);
            } else {
                fly1.setPower(0);
                fly2.setPower(0);
            }
        }

        prevLB = currLB;
        currLB = gamepad1.left_bumper;
        if (currLB && !prevLB) {
            intakeOn = !intakeOn;
            if (intakeOn) {
               intake.setPower(0.8);
            } else {
                intake.setPower(0);
            }
        }

        prevA = currA;
        currA = gamepad1.a;
        if (currA && !prevA) {
            transOn = !transOn;
            if (transOn) {
                transfer.setPower(0.6);
            } else {
                transfer.setPower(0);
            }
        }
        telemetry.addData("Fly Left", fly1.getVelocity());
        telemetry.addData("Fly Right", fly2.getVelocity());
        telemetry.addData("Intake", intake.getVelocity());
        telemetry.addData("Transfer", transfer.getVelocity());
    }
}
