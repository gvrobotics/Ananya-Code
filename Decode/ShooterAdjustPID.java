package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ShooterAdjustw/PID")
public class ShooterAdjustPID extends OpMode {

    // ===== Hardware =====
    DcMotorEx fly1, fly2, intake1, intake2;
    Servo push1, push2, launch;
    Limelight3A limelight;

    // ===== Intake toggle =====
    boolean intakeOn = false;
    boolean prevLB = false, currLB = false;

    // ===== Shooter toggle =====
    boolean shooterOn = false;
    boolean prevRB = false;

    // ===== B button launch =====
    double INTAKEON_TIME = 0.2;
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
    enum LaunchState { IDLE, PUSH_DOWN, PUSH_BACK, INTAKE_ON }
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

        // ===== INTAKE TOGGLE (Left Bumper) =====
        prevLB = currLB;
        currLB = gamepad1.left_bumper;
        if (currLB && !prevLB) {
            intakeOn = !intakeOn;
        }

        // ONLY allow toggle intake when NOT shooting
        if (launchState == LaunchState.IDLE) {
            double intakePower = 0;

            intake1.setPower(intakePower);
            intake2.setPower(intakePower);
        }

        // ================= SHOOTER TOGGLE =================
        boolean currRB = gamepad1.right_bumper;
        if (currRB && !prevRB) shooterOn = !shooterOn;
        prevRB = currRB;

        // ===== FF + P CONTROL LOOP =====
        double velocity = Math.abs(fly1.getVelocity());
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            power = (targetVelocity * kF) + (error * kP);
        }

        fly1.setPower(power);
        fly2.setPower(power);

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

        // ====== LAUNCH SEQUENCE FOR 3 SHOTS (X button) =====
        prevX = currX;
        currX = gamepad1.x;

        if (currX && !prevX && launchState == LaunchState.IDLE) {
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

        // ===== LAUNCH SEQUENCE FOR 1 SHOT (B button) =====
        prevB = currB;
        currB = gamepad1.b;
        // Start launch sequence on B tap if IDLE
        if (currB && !prevB && launchState == LaunchState.IDLE) {
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
                if (timer.seconds() >= 0.2) {
                    // Pusher down
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    timer.reset();
                    launchState = LaunchState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() >= 0.2) {
                    // Turn intake on
                    intake1.setPower(0.8);
                    intake2.setPower(0.8);

                    timer.reset();
                    launchState = LaunchState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() >= INTAKEON_TIME) { // change this value depending on the intake timing issues
                    shotsRemaining--;
                    // Stop intake
                    intake1.setPower(0);
                    intake2.setPower(0);


                    if (shotsRemaining > 0) {
                        push1.setPosition(pushUp1);
                        push2.setPosition(pushUp2);
                        timer.reset();
                        launchState = LaunchState.PUSH_DOWN;
                    } else {
                        launchState = LaunchState.IDLE;
                    }

                    if (shotsRemaining == 2) {
                        INTAKEON_TIME = 0.4;
                    } else {
                        INTAKEON_TIME = 0.2;
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ================= LIMELIGHT DISTANCE =================
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ty = result.getTy();
            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);

            trigDistance =
                    (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);
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
