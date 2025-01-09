package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
       // ElbowL = hardwareMap.get(Servo.class, "EL");
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
       //ElbowL.setDirection(Servo.Direction.FORWARD);
        CArm.setDirection(Servo.Direction.REVERSE);
        CControl.setDirection(Servo.Direction.FORWARD);
        CRotate.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        VSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //MAYBE
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        VSlideR.setPower(0);
        VSlideL.setPower(0);
        ElbowR.setPosition(0.8);
       //ElbowL.setPosition(0);
        CArm.setPosition(0.15);
        CControl.setPosition(0.5);
        CRotate.setPosition(0.71);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad2.left_stick_x;
        powerLY = gamepad2.left_stick_y;
        powerRX = gamepad2.right_stick_x;
        powerRY = gamepad2.right_stick_y;

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

        //INTAKE/OUTTAKE
        //Vertical SLIDES - dpad (up & down)
        //up
        if (gamepad2.dpad_up) {
            VSlideR.setPower(1.0);
            VSlideL.setPower(1.0);
        } else {
            VSlideR.setPower(0);
            VSlideL.setPower(0);
        }

        //down
        if (gamepad2.dpad_down) {
            VSlideR.setPower(-0.6);
            VSlideL.setPower(-0.6);
        } else {
            VSlideR.setPower(0);
            VSlideL.setPower(0);
        }

        //stay up
        if (gamepad2.right_trigger > 0.5) {
            VSlideR.setPower(0.2);
            VSlideL.setPower(0.2);
        }

        //ELBOW - dpad (left & right)
        //out
        if (gamepad2.dpad_right) {
            ElbowR.setPosition(0.7);
            //ElbowL.setPosition(0);
        }
        //in
        if (gamepad2.dpad_left) {
            ElbowR.setPosition(0.9);
            //ElbowL.setPosition(0);
        }

        //CLAW ARM - bumpers
        //up
        if (gamepad2.right_bumper) {
            CArm.setPosition(0.15);
        }
        //down
        if (gamepad2.left_bumper) {
            CArm.setPosition(0.62);
        }

        //CLAW ROTATE - buttons (a & y)
        //straight
        if (gamepad2.y) {
            CRotate.setPosition(0.71);
        }
        //90
        if (gamepad2.a) {
            CRotate.setPosition(0.35);
        }

        //CLAW OPEN & CLOSE - buttons (b & x)
        //open
        if (gamepad2.b) {
            CControl.setPosition(0.2);
        }
        //close
        if (gamepad2.x) {
            CControl.setPosition(0.5);
        }

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addLine();
        telemetry.addData("Vertical Left Slide: ", VSlideL.getPower());
        telemetry.addData("Vertical Right Slide: ", VSlideR.getPower());
        telemetry.addLine();
        //telemetry.addData("Left Elbow: ", ElbowL.getPosition());
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
        telemetry.addLine();
        telemetry.update();
    }
}