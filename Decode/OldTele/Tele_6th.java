package org.firstinspires.ftc.teamcode.Tele;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Dec 6th Tele", group = "Tele")
public class Tele_6th extends OpMode {
    public DcMotor BR, BL, FR, FL, intake, shooter;
    public CRServo spin1, spin2;
    private double powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;
    //private ElapsedTime timer = new ElapsedTime();
    //private Boolean t = false;
    private Boolean motorOn = false, previousGamepad = false, currentGamepad = false;
    private Boolean motorOn2 = false, previousGamepad2 = false, currentGamepad2 = false;
    private Boolean motorOn3 = false, previousGamepad3 = false, currentGamepad3 = false;


    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        intake = hardwareMap.get(DcMotor.class, "i");
        shooter = hardwareMap.get(DcMotor.class, "s");
        spin1 = hardwareMap.get(CRServo.class, "s1");
        spin2 = hardwareMap.get(CRServo.class, "s2");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.REVERSE);
        spin2.setDirection(CRServo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        intake.setPower(0);
        shooter.setPower(0);
        spin1.setPower(0);
        spin2.setPower(0);
    }

    @Override
    public void loop() {
        powerRX = -gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y/1.2;
        powerLX = gamepad1.right_stick_x/1.2;

        robotAngle = Math.atan2(powerLX, powerLY);
        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier * (Math.sin(robotAngle + (Math.PI / 4)))) + powerRX;
        rb = (PowerMultiplier * (Math.sin(robotAngle + (Math.PI / 4)))) - powerRX;
        lb = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) + powerRX;
        rf = (PowerMultiplier * -1 * (Math.sin(robotAngle - (Math.PI / 4)))) - powerRX;

        BR.setPower(rb);
        BL.setPower(lb);
        FR.setPower(rf);
        FL.setPower(lf);

        // TOGGLE FOR INTAKE

        previousGamepad = currentGamepad;
        currentGamepad = gamepad1.left_bumper;

        if (currentGamepad && !previousGamepad) {
            motorOn = !motorOn;
            if (motorOn) {
                intake.setPower(0.7);
            } else {
                intake.setPower(0);
            }
        }


        // TOGGLE FOR SERVO

        previousGamepad3 = currentGamepad3;
        currentGamepad3 = gamepad1.x;

        if (currentGamepad3 && !previousGamepad3) {
            motorOn3 = !motorOn3;
            if (motorOn3) {
                spin1.setPower(0.8);
                spin2.setPower(0.8);
            } else {
                spin1.setPower(0);
                spin2.setPower(0);
            }
        }

        // TOGGLE FOR SHOOTER

        previousGamepad2 = currentGamepad2;
        currentGamepad2 = gamepad1.right_bumper;

        if (currentGamepad2 && !previousGamepad2) {
            motorOn2 = !motorOn2;
            if (motorOn2) {
                shooter.setPower(0.7);
            } else {
                shooter.setPower(0);
            }
        }


        // ELAPSED TIMER CODE

        /*
        // flip
        if (gamepad1.right_bumper && !t) {
            // When right bumper is pressed, flip down
            flip.setPosition(0.2);
            timer.reset();
            t = true;
        } else if (t && !gamepad1.right_bumper && timer.seconds() > 0.3) {
            // After right bumper is released for 0.3s, flip back up
            flip.setPosition(0);
            t = false;
            gamepad1.rumbleBlips(2);
        }
        */

        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("Intake: ", intake.getPower());
        telemetry.addData("Shooter: ", shooter.getPower());
        telemetry.addData("Servo 1: ", spin1.getPower());
        telemetry.addData("Servo 2: ", spin2.getPower());
        telemetry.addData("Intake On: ", motorOn);
        telemetry.addData("Shooter On: ", motorOn2);
        telemetry.addData("Servos On: ", motorOn3);
        telemetry.update();
    }
}
