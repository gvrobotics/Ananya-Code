package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "Tele 5440", group = "A")
public class Tele5440 extends OpMode {
    public DcMotor BR, BL, FR, FL;
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;
    GoBildaPinpointDriver odo;

    private Boolean intakeOn = false, previousGamepad = false, currentGamepad = false;
    private Boolean intakeDirection = false, previousGamepad3 = false, currentGamepad3 = false;

    private Boolean shooterOn = false, previousGamepad2 = false, currentGamepad2 = false;
    double TARGET_VELOCITY = 1050;
    private Boolean launchOn = false, previousGamepad4 = false, currentGamepad4 = false;
    private Boolean pushOn = false;
    private boolean prevY = false, prevA = false, prevB = false;
    private Boolean fieldCentricMode = true;
    private double P = 0.212, F = 12.199;

    // State machine for launch sequence
    private enum LaunchState {
        IDLE,
        PUSH_DOWN,
        PUSH_BACK,
        INTAKE_ON
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Initialize drive motors
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        // Initialize mechanism motors
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");

        // Initialize servos
        push1 = hardwareMap.get(Servo.class, "p1");
        // push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        // push2.setDirection(Servo.Direction.REVERSE);
        launch.setDirection(Servo.Direction.FORWARD);

        // Configure flywheel motors with PIDF
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fly1.setVelocityPIDFCoefficients(P, 0, 0, F);
        fly2.setVelocityPIDFCoefficients(P, 0, 0, F);

        // Set all motors to brake when power is zero
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all motors to zero power
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setVelocity(0);
        intake2.setVelocity(0);
        push1.setPosition(0.5);
        launch.setPosition(0.5);

        // ===== ODOMETRY CONFIGURATION =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Reset and set starting position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    @Override
    public void loop() {
        // MUST be called every loop for field-centric to work
        odo.update();

        // ===== TOGGLE DRIVE =====
        boolean currentY = gamepad1.y;
        if (currentY && !prevY) {
            fieldCentricMode = !fieldCentricMode;
        }
        prevY = currentY;

        // ===== DRIVE CONTROL =====
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double br, bl, fr, fl;

        if (fieldCentricMode) {
            // FIELD-CENTRIC
            // Get robot heading from odometry
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            // Change (Math.PI / 2) to adjust which direction is "field forward"
            double rotationAngle = (0) - heading;
            double cosAngle = Math.cos(rotationAngle);
            double sinAngle = Math.sin(rotationAngle);

            // Rotate the movement vector by the robot's heading
            double globalStrafe = -y * sinAngle + x * cosAngle;
            double globalForward = y * cosAngle + x * sinAngle;

            // Calculate mecanum wheel powers with field-centric adjustments
            fl = globalForward + globalStrafe + rx;
            fr = globalForward - globalStrafe - rx;
            bl = globalForward - globalStrafe + rx;
            br = globalForward + globalStrafe - rx;
        } else {
            // ROBOT-CENTRIC
            fl = y + x + rx;
            fr = y - x - rx;
            bl = y - x + rx;
            br = y + x - rx;
        }

        // Set motor powers
        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);

        // ===== INTAKE TOGGLE (Left Bumper) =====
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

        // ===== SHOOTER TOGGLE (Right Bumper) =====
        previousGamepad2 = currentGamepad2;
        currentGamepad2 = gamepad1.right_bumper;

        if (currentGamepad2 && !previousGamepad2) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                fly1.setVelocity(1500);
                fly2.setVelocity(1500);

                double currentRPM =
                        (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;

                if (currentRPM > TARGET_VELOCITY) {
                    gamepad1.rumble(5);
                }

            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }
        }

        // ===== PUSH TOGGLE (A button) =====
        boolean currentA = gamepad1.a;

        // Only allow A toggle if not in launch sequence
        if (launchState == LaunchState.IDLE) {
            if (currentA && !prevA) {
                pushOn = !pushOn;
                if (pushOn) {
                    push1.setPosition(0.5); // up
                } else {
                    push1.setPosition(0.1); // down
                }
            }
        }
        prevA = currentA;

        // ===== LAUNCH SEQUENCE (B button) =====
        boolean currentB = gamepad1.b;

        // Start launch sequence on B tap if IDLE
        if (currentB && !prevB && launchState == LaunchState.IDLE) {
            // Stop intake
            intake1.setVelocity(0);
            intake2.setVelocity(0);
            intakeOn = false;

            // Pusher up
            push1.setPosition(0.1);

            // Start state machine
            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }
        prevB = currentB;

        // Run launch state machine
        switch (launchState) {
            case PUSH_DOWN:
                if (timer.seconds() >= 0.6) {
                    // Pusher down
                    push1.setPosition(0.5);
                    timer.reset();
                    launchState = LaunchState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() >= 0.6) {
                    // Turn intake on
                    intake1.setVelocity(1500);
                    intake2.setVelocity(1500);
                    timer.reset();
                    launchState = LaunchState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() >= 0.2) {
                    // Stop intake
                    intake1.setVelocity(0);
                    intake2.setVelocity(0);
                    launchState = LaunchState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ===== LAUNCH TOGGLE (X button) =====
        previousGamepad4 = currentGamepad4;
        currentGamepad4 = gamepad1.x;

        if (currentGamepad4 && !previousGamepad4) {
            launchOn = !launchOn;
            if (launchOn) {
                launch.setPosition(1);
            } else {
                launch.setPosition(0.5);
            }
        }

        previousGamepad3 = currentGamepad3;
        currentGamepad3 = gamepad1.dpad_down;

        if (currentGamepad3 && !previousGamepad3) {
            intakeDirection = !intakeDirection;
            if (intakeDirection) {
                intake1.setDirection(DcMotorSimple.Direction.REVERSE);
                intake2.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                intake1.setDirection(DcMotorSimple.Direction.FORWARD);
                intake2.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        // ===== TELEMETRY =====
        // Drive mode indicator
        telemetry.addData("DRIVE MODE", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        //telemetry.addData("Alliance", color ? "RED" : "BLUE");
        telemetry.addLine();

        telemetry.addLine("========STATE========");
        telemetry.addData("Flywheel", shooterOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Intake Direction (Dpad Down)", intakeDirection ? "Reverse" : "Forward");
        telemetry.addData("Push (a)", pushOn ? "Down" : "Up");
        telemetry.addData("Angle", launch.getPosition());

        // telemetry.addData("Launch (x)", launchOn ? "Far" : "Close");

        telemetry.addLine();
        telemetry.addLine("========VALUES========");
        telemetry.addData("Fly Up", fly2.getVelocity());
        telemetry.addData("Fly Down", fly1.getVelocity());
        telemetry.addData("Fly1p", fly1.getPower());
        telemetry.addData("Fly2p", fly2.getPower());
        telemetry.addLine();

        // Intake info
        telemetry.addData("Intake1", intake1.getPower());
        telemetry.addData("Intake2", intake2.getPower());
        telemetry.addData("Push1", push1.getPosition());
        telemetry.addLine();

        // Drive motor info
        // Odometry position (only shown in field-centric mode)
        if (fieldCentricMode) {
            Pose2D pos = odo.getPosition();
            telemetry.addData("Robot X", pos.getX(DistanceUnit.MM));
            telemetry.addData("Robot Y", pos.getY(DistanceUnit.MM));
            telemetry.addData("Robot Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
        }
        telemetry.addData("BR", BR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("FL", FL.getPower());
        telemetry.addData("powerRX", gamepad1.right_stick_x);
        telemetry.addData("powerLX", gamepad1.left_stick_x);
        telemetry.addData("powerLY", gamepad1.left_stick_y);
        telemetry.addLine();
        telemetry.update();
    }
}

// Replace (Math.PI / 2) with:
//     0           -> Field forward = 0° (red alliance wall in FTC)
//     Math.PI/2   -> Field forward = 90°
//     Math.PI     -> Field forward = 180° (blue alliance wall)
//     3*Math.PI/2 -> Field forward = 270°
