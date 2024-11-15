package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Tele5440 extends OpMode {
    private double powerLX, powerLY, powerRX, powerRY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    private DcMotor FL, BL, FR, BR, Lift, Slides1, Slides2;
    private Servo Claw;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Slides1 = hardwareMap.get(DcMotor.class, "Slides1");
        Slides2 = hardwareMap.get(DcMotor.class, "Slides2");
        Lift = hardwareMap.get(DcMotor.class, "Lift");

        Claw = hardwareMap.get(Servo.class, "Claw");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides1.setDirection(DcMotorSimple.Direction.FORWARD);
        Slides2.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Claw.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        Slides1.setPower(0);
        Slides2.setPower(0);
        Lift.setPower(0);
        Claw.setPosition(0);
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

        lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) - powerRX;
        rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) + powerRX;

        FR.setPower(rf);
        FL.setPower(lf);
        BR.setPower(rb);
        BL.setPower(lb);

        //INTAKE/OUTAKE
        //SLIDES - dpad
        //up
        if (gamepad2.dpad_up) {
            Slides1.setPower(0.3);
            Slides2.setPower(0.3);
        } else {
            Slides1.setPower(0);
            Slides2.setPower(0);
        //down
        if (gamepad2.dpad_down)    {
            Slides1.setPower(-0.3);
            Slides2.setPower(-0.3);
        } else {
            Slides1.setPower(0);
            Slides2.setPower(0);
        }

        //SLIDES - right stick
        //up
//       if (gamepad2.right_stick_y > 0.05) {
//       Slide1.setPower(0.3);
//       Slide2.setPower(0.3);
//       } else {
//       Slide1.setPower(0);
//       Slide2.setPower(0);
//       }
//       //down
//       if (gamepad2.right_stick_y < -0.05)    {
//       Slide1.setPower(-0.3);
//       Slide2.setPower(-0.3);
//       } else {
//       Slide1.setPower(0);
//       Slide2.setPower(0);
//       }

        //ARM - dpad
        //up
        if (gamepad2.dpad_left) {
            Lift.setPower(0.5);
        } else {
            Lift.setPower(0);
        }
        //down
        if (gamepad2.dpad_right) {
            Lift.setPower(-0.5);
        } else {
            Lift.setPower(0);
        }

        //LIFT - stick
        //up
//        if (gamepad2.left_stick_y > 0.05) {
//            Lift.setPower(0.5);
//       } else {
//            Lift.setPower(0);
//       }
//       //down
//       if (gamepad2.left_stick_y < -0.05) {
//           Lift.setPower(-0.5);
//       } else {
//           Lift.setPower(0);
//       }

        //CLAW
        //open claw
        if (gamepad2.y) {
        Claw.setPosition(0.4);
        }
        //close claw
        if (gamepad2.a) {
        Claw.setPosition(0);
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

        telemetry.addData("Slides1: ", Slides1.getPower());
        telemetry.addData("Slides2: ", Slides2.getPower());
        telemetry.addData("Lift: ", Lift.getPower());
        telemetry.addData("Claw: ", Claw.getPosition());

        telemetry.update();
        }
    }
}