package org.firstinspires.ftc.teamcode.Tele.Test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tele.Tele5440;

@TeleOp(name = "ShooterAdjustPID", group = "C")
public class ShooterAdjustPID extends OpMode {

    // ===== Hardware =====
    DcMotorEx fly1, fly2, intake1, intake2;
    Servo push1, push2, launch;
    Limelight3A limelight;

    // ===== Intake toggle =====
    boolean intakeOn = false;
    boolean prevLB1 = false, currLB1 = false;

    // ===== Shooter toggle =====
    boolean shooterOn = false;
    boolean prevRB = false;

    // ===== B button launch =====
    double INTAKEON_TIME = 0.15, PUSHED_UP_TIME = 0.15, PUSHED_DOWN_TIME = 0.05;
    boolean prevB = false, currB = false;

    // ===== D-pad edge detect =====
    boolean prevUp = false, prevDown = false;
    boolean prevX = false, currX = false;

    // ===== Shooter velocity =====
    double targetVelocity = 1300;

    // ===== FF + P CONTROL VALUES =====
    double kF = 0.00052;
    double kP = 0.0026;

    // ===== Launch state machine =====
    enum LaunchState { IDLE, PUSH_UP, PUSH_DOWN, PUSH_BACK, INTAKE_OFF }
    LaunchState launchState = LaunchState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    int shotsRemaining = 0;

    // ===== Servo positions =====
    double pushUp1 = 0.7, pushDown1 = 0.4;
    double pushUp2 = 0.25, pushDown2 = 0;

    // ===== Limelight constants =====
    static final double LIME_MOUNT_ANGLE = 15;
    static final double LIME_HEIGHT = 13;
    static final double TAG_HEIGHT = 38;

    double trigDistance = -1;

    @Override
    public void init() {

        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");

        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        // Configure flywheel motors - MANUAL CONTROL FOR FF+P
        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        push1.setPosition(pushDown1);
        push2.setPosition(pushDown2);
        launch.setPosition(0.5);
    }

    @Override
    public void loop() {
        // ================= LIMELIGHT DISTANCE =================
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ty = result.getTy();
            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);

            trigDistance =
                    (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);
        }

        // ================= VELOCITY ADJUST =================
        boolean currUp = gamepad1.dpad_up;
        boolean currDown = gamepad1.dpad_down;

        if (currUp && !prevUp) targetVelocity += 10;
        if (currDown && !prevDown) targetVelocity -= 10;

        prevUp = currUp;
        prevDown = currDown;

        // ================= LAUNCH ANGLE ADJUST =================
        if (gamepad1.dpadLeftWasPressed())
            launch.setPosition(launch.getPosition() + 0.05);

        if (gamepad1.dpadRightWasPressed())
            launch.setPosition(launch.getPosition() - 0.05);

        // ===== FF + P CONTROL LOOP =====
        double velocity = Math.abs(fly1.getVelocity());
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            // Calculate base power using existing constants
            power = (targetVelocity * kF) + (error * kP);
        }

        fly1.setPower(power);
        fly2.setPower(power);

        // ===== INTAKE TOGGLE (Left Bumper) =====
        prevLB1 = currLB1;
        currLB1 = gamepad1.left_bumper;
        if (currLB1 && !prevLB1) {
            intakeOn = !intakeOn;
        }

        // ONLY allow toggle intake when NOT shooting
        if (launchState == LaunchState.IDLE) {
            double intakePower = 0;
            if (intakeOn) {
                intake1.setPower(intakePower);
                intake2.setPower(intakePower);
            }
        }

        // ====== LAUNCH SEQUENCE FOR 3 SHOTS (B button) =====
        prevB = currB;
        currB = gamepad1.b;

        if (currB && !prevB && launchState == LaunchState.PUSH_UP) {
            shotsRemaining = 3;

            // Stop intake
            intake1.setPower(0);
            intake2.setPower(0);
            intakeOn = false;

            // Pusher up
            push1.setPosition(pushUp1);
            push2.setPosition(pushUp2);

            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }

        // ===== LAUNCH SEQUENCE FOR 1 SHOT (X button) =====
        prevX = currX;
        currX = gamepad1.x;
        // Start launch sequence on B tap if IDLE
        if (currX && !prevX && launchState == LaunchState.PUSH_UP) {
            shotsRemaining = 1;
            // Stop intake
            intake1.setPower(0);
            intake2.setPower(0);

            intakeOn = false;

            // Pusher up
            push1.setPosition(pushUp1);
            push2.setPosition(pushUp2);

            // Start state machine
            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }

        // Run launch state machine
        switch (launchState) {
            case PUSH_DOWN:
                // if pusher is up for PUSHED_UP_TIME push down transfer
                if (timer.seconds() >= PUSHED_UP_TIME) {
                    // Pusher down
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    timer.reset();
                    launchState = LaunchState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                // if pusher is down for PUSHED_DOWN_TIME turn on intake
                if (timer.seconds() >= PUSHED_DOWN_TIME) {
                    // Turn intake on to load next sample
                    intake1.setPower(0.8);
                    intake2.setPower(0.8);

                    timer.reset();
                    launchState = LaunchState.INTAKE_OFF;
                }
                break;

            case INTAKE_OFF:
                // if intake is on for INTAKEON_TIME turn it off
                if (timer.seconds() >= INTAKEON_TIME) {
                    shotsRemaining--;
                    // Stop intake
                    intake1.setPower(0);
                    intake2.setPower(0);

                    // if more artifacts remain, rerun launch automations
                    if (shotsRemaining > 0) {
                        push1.setPosition(pushUp1);
                        push2.setPosition(pushUp2);
                        timer.reset();
                        launchState = LaunchState.PUSH_DOWN;
                    } else {
                        launchState = LaunchState.PUSH_UP;
                    }

//                    if (shotsRemaining == 2) {
//                        INTAKEON_TIME = 0.3;
//                    } else {
//                        INTAKEON_TIME = 0.15;
//                    }
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ================= TELEMETRY =================
        telemetry.addData("Distance (in)", trigDistance);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Fly Up", fly1.getVelocity());
        telemetry.addData("Fly Down", fly2.getVelocity());
        telemetry.addData("Launch Angle", launch.getPosition());
//        telemetry.addData("Error", error);
//        telemetry.addData("Power", power);
        telemetry.addData("Intake Power", intake1.getPower());
        telemetry.update();
    }
}
