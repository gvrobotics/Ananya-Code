package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Tele5440")
public class Tele5440 extends OpMode
{
    public DcMotor FR, FL, BR, BL, VSlideR, VSlideL;
    private Servo CControl, CRotate, CArm, ElbowR, ElbowL;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;

    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        VSlideR = hardwareMap.get(DcMotor.class, "VSR");
        VSlideL = hardwareMap.get(DcMotor.class, "VSL");
        ElbowR = hardwareMap.get(Servo.class, "ER");
        ElbowL = hardwareMap.get(Servo.class, "EL");
        CArm = hardwareMap.get(Servo.class, "CA");
        CControl = hardwareMap.get(Servo.class, "CC");
        CRotate = hardwareMap.get(Servo.class, "CR");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        VSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        VSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        ElbowR.setDirection(Servo.Direction.FORWARD);
        ElbowL.setDirection(Servo.Direction.FORWARD);
        CArm.setDirection(Servo.Direction.REVERSE);
        CRotate.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        VSlideR.setPower(0.2);
        VSlideL.setPower(0);
        ElbowL.setPosition(0.1);
        ElbowR.setPosition(0.45);
        CArm.setPosition(0.3);
        CControl.setPosition(0.5);
        CRotate.setPosition(0.71);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;
        powerRY = gamepad1.right_stick_y;

        robotAngle = Math.atan2(powerLX, powerLY);

        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) - powerRX;
        rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) + powerRX;

        BR.setPower(rb);
        BL.setPower(lb);
        FR.setPower(rf);
        FL.setPower(lf);

    //Vertical SLIDES - dpad (up & down)
    //up
    if (gamepad2.dpad_up) {
        VSlideR.setPower(0.7);
        VSlideL.setPower(0.7);
    //down
    } else if (gamepad2.dpad_down) {
        VSlideR.setPower(-0.3);
        VSlideL.setPower(-0.3);
    //brake
    } else if (gamepad2.right_trigger > 0) {
        VSlideR.setPower(0.2);
        VSlideL.setPower(0.2);
    //max power up
    } else if(gamepad2.left_trigger > 0) {
        VSlideR.setPower(1.0);
        VSlideL.setPower(1.0);
    //max power down
    } else if (gamepad2.back) {
        VSlideR.setPower(-1.0);
        VSlideL.setPower(-1.0);
    } else {
        VSlideR.setPower(0);
        VSlideL.setPower(0);
    }

    //ELBOW - dpad (left & right)
    //out
    if (gamepad2.dpad_left) {
        ElbowL.setPosition(0.45);
        ElbowR.setPosition(0.6);
    //in
    } else if (gamepad2.dpad_right) {
        ElbowL.setPosition(0.1);
        ElbowR.setPosition(0.3);
    }

    //CLAW ARM - bumpers
    //up
    if (gamepad2.left_bumper) {
        CArm.setPosition(0.35);
    //down
    } else if (gamepad2.right_bumper) {
        CArm.setPosition(0.9);
    //straight
    } else if (gamepad2.start) {
        CArm.setPosition(0.6);
    }

    //CLAW ROTATE - buttons (a & y)
    //straight
    if (gamepad2.y) {
        CRotate.setPosition(0.71);
    //90
    } else if (gamepad2.a) {
        CRotate.setPosition(0.35);
    }

    //CLAW OPEN & CLOSE - buttons (b & x)
    //open
    if (gamepad2.b) {
        CControl.setPosition(1.0);
    //close
    } else if (gamepad2.x) {
        CControl.setPosition(0.7);
    }

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addLine();
        telemetry.addData("Vertical Left Slide: ", VSlideL.getPower());
        telemetry.addData("Vertical Right Slide: ", VSlideR.getPower());
        telemetry.addLine();
        telemetry.addData("Left Elbow: ", ElbowL.getPosition());
        telemetry.addData("Right Elbow: ", ElbowR.getPosition());
        telemetry.addLine();
        telemetry.addData("Claw Arm: ", CArm.getPosition());
        telemetry.addData("Claw Rotate: ", CRotate.getPosition());
        telemetry.addData("Claw Open & Close: ", CControl.getPosition());
        telemetry.addLine();
        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);
        telemetry.update();
    }
}