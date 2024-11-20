package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class Tele5440 extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR, Arm, Slides;
    private CRServo Spin;
    //private double CPD = 4; //old arm motor
    private double CPD = 6; //new arm motor
    double armPosition, armPositionFudgeFactor;

    /*
     28 // number of encoder ticks per rotation of the bare motor
   * 19.2 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
   * 90.0 / 30.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
   * 1 / 360.0; // we want ticks per degree, not per rotation
    */
    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Slides = hardwareMap.get(DcMotor.class, "Slides1");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        Spin = hardwareMap.get(CRServo.class, "Spin");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides.setDirection(DcMotorSimple.Direction.FORWARD);
        //Arm.setDirection(DcMotorSimple.Direction.FORWARD);

        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Slides.setPower(0);
        //Arm.setPower(0);
        Spin.setPower(0);
    }

    @Override
    public void loop() {
        powerLX = gamepad1.left_stick_x / 2;
        powerLY = gamepad1.left_stick_y / 2;
        powerRX = gamepad1.right_stick_x / 2;
        powerRY = gamepad1.right_stick_y / 2;

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
        //SLIDES - bumpers
        //up
        if (gamepad2.right_bumper) {
            Slides.setPower(0.3);
        } else {
            Slides.setPower(0);
        }
        //down
        if (gamepad2.left_bumper) {
            Slides.setPower(-0.3);
        } else {
            Slides.setPower(0);
        }

        //ARM - dpad
        //up
        if (gamepad2.dpad_up) {
            armPosition = 40 * CPD;
            Arm.setTargetPosition((int) (armPosition));
        } else {
            Arm.setPower(0);
        }
        //down
        if (gamepad2.dpad_down) {
            armPosition = 0;
            Arm.setTargetPosition((int) (armPosition));
        } else {
            Arm.setPower(0);
        }

        armPositionFudgeFactor = (15 * CPD) * (gamepad1.right_trigger + (-gamepad1.left_trigger));

        //Spin
        //Collect
        if (gamepad2.x) {
            Spin.setPower(0.5);
        }
        //Deposit
        if (gamepad2.b) {
            Spin.setPower(-1.0);
        }
        //Off
        if (gamepad2.y) {
            Spin.setPower(0);
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
        telemetry.addData("Arm: ", Arm.getPower());
        telemetry.addData("Spin: ", Spin.getPower());

        telemetry.update();
    }
}

//SLIDES - right stick
//up
//       if (gamepad2.right_stick_y > 0.05) {
//       Slide1.setPower(right_stick_y);
//       Slide2.setPower(right_stick_y);
//       } else {
//       Slide1.setPower(0);
//       Slide2.setPower(0);
//       }
//       //down
//       if (gamepad2.right_stick_y < -0.05)    {
//       Slide1.setPower(-right_stick_y);
//       Slide2.setPower(-right_stick_y);
//       } else {
//       Slide1.setPower(0);
//       Slide2.setPower(0);
//       }

//Arm - stick
//up
//        if (gamepad2.left_stick_y > 0.05) {
//            Arm.setPower(left_stick_y);
//       } else {
//            Arm.setPower(0);
//       }
//       //down
//       if (gamepad2.left_stick_y < -0.05) {
//           Arm.setPower(-left_stick_y);
//       } else {
//           Arm.setPower(0);
//       }