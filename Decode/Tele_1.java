package org.firstinspires.ftc.teamcode.Tele;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Tele 1", group = "C")
public class Tele_1 extends OpMode {
    public DcMotor BR, BL, FR, FL;
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;
    private Boolean intakeOn = false, previousGamepad = false, currentGamepad = false, t = false;
    private Boolean shooterOn = false, previousGamepad2 = false, currentGamepad2 = false;
    private Boolean pushOn = false, previousGamepad3 = false, currentGamepad3 = false;
    private Boolean launchOn = false, previousGamepad4 = false, currentGamepad4 = false;


    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private Boolean t2 = false;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");
        push1 = hardwareMap.get(Servo.class, "p1");
        // push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        // push2.setDirection(Servo.Direction.REVERSE);
        launch.setDirection(Servo.Direction.FORWARD);

        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(0.2, 0, 0, 14);
        fly1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        fly2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        push1.setPosition(0.5);
        launch.setPosition(0.9);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double br = (y + x - rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double fl = (y + x + rx) / denominator;

        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);

        // TOGGLE FOR Intake
        previousGamepad = currentGamepad;
        currentGamepad = gamepad1.left_bumper;

        if (currentGamepad && !previousGamepad) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                intake1.setVelocity(1500);
                intake2.setVelocity(1500);
            } else {
                intake1.setVelocity(0);
                intake2.setVelocity(0);
            }
        }

        // TOGGLE FOR Shooter
        previousGamepad2 = currentGamepad2;
        currentGamepad2 = gamepad1.right_bumper;

        if (currentGamepad2 && !previousGamepad2) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                fly1.setVelocity(1500);
                fly2.setVelocity(1500);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }
        }

        // TOGGLE FOR Push - a
        previousGamepad3 = currentGamepad3;
        currentGamepad3 = gamepad1.a;

        if (currentGamepad3 && !previousGamepad3) {
            pushOn = !pushOn;
            if (pushOn) {
                push1.setPosition(0.5); //down
            } else {
                push1.setPosition(0.1); //up
            }
        }

        // LAUNCH - b
        if (gamepad1.b && !t) {
            // When b is pressed, flip up and intake off
            push1.setPosition(0.1);
            intake1.setVelocity(0);
            intake2.setVelocity(0);
            timer.reset();
            t = true;
        } else if (t && !gamepad1.b && timer.seconds() > 2) {
            // After b is released for 2s, flip back down and intake on
            push1.setPosition(0.5);
            intake1.setVelocity(1500);
            intake2.setVelocity(1500);
            timer2.reset();
            t2 = true;
            t = false;
        }
        // turning off intake after 2s
        if (t2 && timer2.seconds() > 2) {
            intake1.setVelocity(0);
            intake2.setVelocity(0);
            t2 = false;
        }

        // TOGGLE FOR Launch - b
        previousGamepad4 = currentGamepad4;
        currentGamepad4 = gamepad1.x;

        if (currentGamepad4 && !previousGamepad4) {
            launchOn = !launchOn;
            if (launchOn) {
                launch.setPosition(0.9);
            } else {
                launch.setPosition(0.3);
            }
        }


        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("Fly1: ", fly1.getVelocity());
        telemetry.addData("Fly2: ", fly2.getVelocity());
        telemetry.addData("Flywheel On: ", shooterOn);
        telemetry.addData("Intake1: ", intake1.getVelocity());
        telemetry.addData("Intake2: ", intake2.getVelocity());
        telemetry.addData("Intake On: ", intakeOn);
        telemetry.addData("Push1: ", push1.getPosition());
        // telemetry.addData("Push2: ", push2.getPosition());
        telemetry.addData("Launch: ", launch.getPosition());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.update();
    }
}
