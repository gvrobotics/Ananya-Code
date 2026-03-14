package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Test.DistanceSensor;


@TeleOp (name = "Tele 5440", group = "A")
public class Tele5440 extends OpMode {
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;
    GoBildaPinpointDriver odo;
    VoltageSensor voltage;
    DistanceSensor distanceSensor = new DistanceSensor();

    private Boolean intakeOn = false, prevLB1 = false, currLB1,
            intakeDirection = false, prevLB2 = false, currLB2,
            shooterOn = false, prevRB1 = false, currRB1,
            pushOn = false, prevA2 = false, currA2,
            prevB = false, currB, prevX = false, currX,
            prevA = false, currA = false,
            prevDL = false, currDL = false,
            prevDR = false, currDR = false;

    private double pushUp1 = 0.7, pushDown1 = 0.4, pushUp2 = 0.25, pushDown2 = 0;
    private int shotsRemaining = 0;
    double intakePower = 0, INTAKEON_TIME = 0.15, PUSHED_UP_TIME = 0.15, PUSHED_DOWN_TIME = 0.05;
    private boolean autoAlignOn = false;
    private boolean wasAligned = false;  // Track alignment state for rumble
    double kP_align = 0.0201; //TODO: ADJUST

    // ================= LIMELIGHT VARIABLES =================
    private Limelight3A limelight;
    private static final double LIME_MOUNT_ANGLE = 15.0, LIME_HEIGHT = 13.0, TAG_HEIGHT = 38.0;
    private double trigDistance = -1;

    // ================= LOOKUP TABLE =================
    private final double [] distances = {30, 40, 50, 60, 75, 137, 165, 172};
    private final double [] angles = {1, 0.7, 0.65, 0.6, 0.37, 0.15, 0.15, 0.1};
    private final double [] velocities = {820, 860, 880, 900, 1020, 1160, 1220, 1230};

    private double targetAngle = 0.5;
    private double targetVelocity = 1000;

    // ===== FF + P CONTROL VALUES =====
    private double kF = 0.00052,  kP = 0.0026;

    // ================= DISTANCE SENSOR =================
    private boolean cdsCheck = false;
    private ElapsedTime cdsTimer = new ElapsedTime();

    // State machine for launch sequence
    private enum LaunchState {
        IDLE,
        PUSH_DOWN,
        PUSH_BACK,
        INTAKE_OFF
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime timer = new ElapsedTime();
    Drivetrain robot;

    @Override
    public void init() {
        // Initialize
        robot = new Drivetrain(hardwareMap);
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");

        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        voltage = hardwareMap.voltageSensor.iterator().next();
        distanceSensor.init(hardwareMap);

        // Set motor directions
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        // Configure flywheel motors
        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all motors to zero power
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        push1.setPosition(pushDown1);
        push2.setPosition(pushDown2);
        launch.setPosition(launch.getPosition());

        // ===== Limelight =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private double interpolate (double [] x, double [] y, double value) {
        //clamp to range
        if (value <= x[0]) return y[0];
        if (value >= x[x.length - 1]) return y[y.length - 1];

        for (int i = 0; i < x.length - 1; i++) {
            if (value >= x[i] && value <= x[i + 1]) {  // for a number that is in range
                double ratio = (value - x[i]) / (x[i + 1] - x[i]); // find ratio of range to value in range

                return y[i] + ratio * (y[i + 1] - y[i]);
            }
        }
        return y[0];
    }

    @Override
    public void loop() {
        // ================= LIMELIGHT DISTANCE =================
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);
            trigDistance = (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);

            // lookup shooter targets
            targetAngle = interpolate(distances, angles, trigDistance);
            targetVelocity = interpolate(distances, velocities, trigDistance);
        }

        // ======== AUTO ALIGN TOGGLE (A button) ========
        prevA = currA;
        currA = gamepad1.a;
        if (currA && !prevA) {
            autoAlignOn = !autoAlignOn;
        }

        // ========= AUTO ALIGN TO TARGET =========
        double rotationCorrection = 0;
        boolean isAligned = false;

        if (autoAlignOn && result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal angle offset

            // Check if robot is aligned (within acceptable threshold)
            isAligned = Math.abs(tx) < 2; // degrees - adjust this value as needed

            // Rumble when newly aligned
            if (isAligned && !wasAligned) {
                gamepad1.rumble(300);
            } else {
                gamepad1.stopRumble();
            }

            // If tx is positive, target is to the right, so rotate right
            // If tx is negative, target is to the left, so rotate left
            rotationCorrection = tx * kP_align;

            // Clamp to reasonable rotation speed
            rotationCorrection = Math.max(-0.3, Math.min(0.3, rotationCorrection));
        }

        wasAligned = isAligned;

        // Pass this to drive
        robot.update(gamepad1, rotationCorrection);

        // ============= SHOOTER TOGGLE (Right Bumper) ===========
        prevRB1 = currRB1;
        currRB1 = gamepad1.right_bumper;
        if (currRB1 && !prevRB1) {
            shooterOn = !shooterOn;
        }

        // ===== FF + P CONTROL LOOP =====
        double velocity = Math.abs(fly1.getVelocity());
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            // Auto aim if limelight has valid target
            if (trigDistance > 0) {
                launch.setPosition(targetAngle);
            }

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
            intakePower = 0;
            if (intakeOn) {
                double basePower = 0.8;
                // Scale intake power with voltage (85%)
                double voltageScale = Math.max(0.85, voltage.getVoltage() / 13);
                double adjustedPower = basePower * voltageScale;

                intakePower = intakeDirection ? -adjustedPower : adjustedPower;
            }

            intake1.setPower(intakePower);
            intake2.setPower(intakePower);
        }

        // ====== LAUNCH SEQUENCE FOR 3 SHOTS (B button) =====
        prevB = currB;
        currB = gamepad1.b;

        if (currB && !prevB && launchState == LaunchState.IDLE) {
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

        if (currX && !prevX && launchState == LaunchState.IDLE) {
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

                    // if more shots remain, rerun launch sequence
                    if (shotsRemaining > 0) {
                        push1.setPosition(pushUp1);
                        push2.setPosition(pushUp2);
                        timer.reset();
                        launchState = LaunchState.PUSH_DOWN;
                    } else {
                        launchState = LaunchState.IDLE;
                    }

                    if (shotsRemaining == 2) {
                        INTAKEON_TIME = 0.3;
                    } else {
                        INTAKEON_TIME = 0.15;
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ===== INTAKE DIRECTION TOGGLE (Left bumper on 2) =====
        prevLB2 = currLB2;
        currLB2 = gamepad2.left_bumper;

        if (currLB2 && !prevLB2) {
            intakeDirection = !intakeDirection;
        }

        // ===== DISTANCE SENSOR =====
        double distance = distanceSensor.getDistance();
        if (distance < 10) {
            if (!cdsCheck) {
                cdsTimer.reset();
                cdsCheck = true;
            }

            if (cdsTimer.seconds() >= 0.2) {
                gamepad1.rumbleBlips(2);
                telemetry.addData("Status: ", "FULL");
            } else {
                gamepad1.stopRumble();
                telemetry.addData("Status: ", "WAITING");
            }
        } else {
            cdsCheck = false;
            gamepad1.stopRumble();
            telemetry.addData("Status: ", "NOT FULL");
        }


        // ===== TELEMETRY =====
        // Drive mode indicator
        telemetry.addData("DRIVE MODE", robot.isFieldCentric() ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addLine();

        telemetry.addLine("========STATE========");
        telemetry.addData("Battery", voltage.getVoltage());
        if (power >= 1.0 && velocity < (targetVelocity * 0.9)) {
            // battery is too low to make shots
            telemetry.addLine("BATTERY IS TOO LOW");
        }

        telemetry.addData("Flywheel", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter Ready", fly1.getVelocity() >= targetVelocity ? "YES" : "NO");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Intake Direction", intakeDirection ? "REVERSE" : "FORWARD");
        telemetry.addLine();

        telemetry.addLine("===== AUTO ALIGN =====");
        telemetry.addData("Auto Align", autoAlignOn ? "ON" : "OFF");
        telemetry.addData("Aligned", isAligned ? "YES" : "NO");
        telemetry.addLine();

        telemetry.addLine("===== AUTO ADJUST =====");
        telemetry.addData("Distance (in)", trigDistance);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addLine();

        telemetry.addLine("========SHOOTER========");
        telemetry.addData("Fly Up", fly1.getVelocity());
        telemetry.addData("Fly Down", fly2.getVelocity());
        telemetry.addData("Power", power);
        telemetry.addLine();

        telemetry.addLine("========VALUES========");
        telemetry.addData("Angle", launch.getPosition());
        telemetry.addData("Intake1", intake1.getPower());
        telemetry.addData("Intake2", intake2.getPower());
        telemetry.addLine();

        // Drive motor info
        // Odometry position (only shown in field-centric mode)
        if (robot.isFieldCentric()) {
            Pose2D pos = robot.getPose();
            telemetry.addData("Robot X", pos.getX(DistanceUnit.MM));
            telemetry.addData("Robot Y", pos.getY(DistanceUnit.MM));
            telemetry.addData("Robot Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
        }
        telemetry.update();
    }
}