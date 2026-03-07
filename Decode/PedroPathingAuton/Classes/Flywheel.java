package org.firstinspires.ftc.teamcode.PedroPathingAuton.Classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    // Hardware
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;
    private Limelight3A limelight;

    // State Machine
    private enum FlywheelState {
        IDLE,
        PUSH_UP,
        PUSH_DOWN,
        PUSH_BACK,
        INTAKE_OFF
    }

    private FlywheelState flywheelState = FlywheelState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    // ================= PUSHER CONSTANTS =================
    private final double PUSH_UP_1 = 0.7, PUSH_DOWN_1 = 0.4, PUSH_UP_2 = 0.25, PUSH_DOWN_2 = 0;
    private final double PUSH_BACK_TIME = 0.2;
    private double intakeOnTime = 0.2;

    // ================= FLYWHEEL CONTROL =================
    private int shotsRemaining = 0;
    private final double kF = 0.00052,  kP = 0.0026;
    private double targetVelocity, targetAngle;

    // ================= LIMELIGHT CONSTANTS =================
    private static final double LIME_MOUNT_ANGLE = 15.0, LIME_HEIGHT = 13.0, TAG_HEIGHT = 38.0;
    private double trigDistance = -1;
    private boolean autoAdjustEnabled = false;

    // ================= LOOKUP TABLES =================
    private final double[] distances = {30, 40, 50, 60, 75, 137, 165, 172};
    private final double[] angles = {1, 0.7, 0.65, 0.6, 0.35, 0.15, 0.15, 0.1};
    private final double[] velocities = {820, 860, 880, 900, 1020, 1160, 1220, 1230};

    // ================= ALIGNMENT =================
    private double targetTx = 0; // Target horizontal offset (0 = centered)
    private static final double ALIGNMENT_THRESHOLD = 2.0; // degrees - adjust based on testing

    public void init(HardwareMap hwMap) {
        // Motors
        fly1 = hwMap.get(DcMotorEx.class, "f1");
        fly2 = hwMap.get(DcMotorEx.class, "f2");
        intake1 = hwMap.get(DcMotorEx.class, "i1");
        intake2 = hwMap.get(DcMotorEx.class, "i2");

        // Servos
        push1 = hwMap.get(Servo.class, "p1");
        push2 = hwMap.get(Servo.class, "p2");
        launch = hwMap.get(Servo.class, "l");

        // Limelight
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Motor directions
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo directions
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        // Motor modes
        fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero power behavior
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize positions
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        push1.setPosition(PUSH_DOWN_1);
        push2.setPosition(PUSH_DOWN_2);
        launch.setPosition(0.7);
    }

    // =========== Linear interpolation ==============
    private double interpolate(double[] x, double[] y, double value) {
        // Clamp to range
        if (value <= x[0]) return y[0];
        if (value >= x[x.length - 1]) return y[y.length - 1];

        // Find interpolation range
        for (int i = 0; i < x.length - 1; i++) {
            if (value >= x[i] && value <= x[i + 1]) {
                double ratio = (value - x[i]) / (x[i + 1] - x[i]);
                return y[i] + ratio * (y[i + 1] - y[i]);
            }
        }
        return y[0];
    }

    // ================= LIMELIGHT DISTANCE =================
    public void updateLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);
            trigDistance = (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);

            // Auto-calculate targets from distance if enabled
            if (autoAdjustEnabled) {
                targetAngle = interpolate(distances, angles, trigDistance);
                targetVelocity = interpolate(distances, velocities, trigDistance);
            }
        } else {
            trigDistance = -1; // Invalid reading
        }
    }

    // ================= STATE MACHINE UPDATE =================
    public void update() {
        // Update limelight data
        updateLimelight();

        // Run flywheel control
        updateFlywheelPower();

        // State machine
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    // Reset pusher and start spinning up
                    push1.setPosition(PUSH_DOWN_1);
                    push2.setPosition(PUSH_DOWN_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_UP;
                }
                break;

            case PUSH_UP:
                double currentRPM = getFlywheelRPM();

                // update launch angle while spinning up
                if (autoAdjustEnabled && trigDistance > 0) {
                    launch.setPosition(targetAngle);
                }

                // Wait until flywheel is at target velocity
                if (currentRPM >= targetVelocity * 0.95) { // 5% tolerance
                    // Push up to launch
                    push1.setPosition(PUSH_UP_1);
                    push2.setPosition(PUSH_UP_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_DOWN;
                }
                break;

            case PUSH_DOWN:
                if (stateTimer.seconds() >= 0.15) {
                    shotsRemaining--;

                    // Push down
                    push1.setPosition(PUSH_DOWN_1);
                    push2.setPosition(PUSH_DOWN_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_BACK;
                }
                break;

//            case PUSH_DOWN:
//                if (stateTimer.seconds() >= 0.2) {
//                    stateTimer.reset();
//                    flywheelState = FlywheelState.PUSH_BACK;
//                }
//                break;

            case PUSH_BACK:
                if (stateTimer.seconds() >= PUSH_BACK_TIME) {
                    // Turn on intake to load next sample
                    intake1.setPower(0.8);
                    intake2.setPower(0.8);
                    stateTimer.reset();
                    flywheelState = FlywheelState.INTAKE_OFF;
                }
                break;

            case INTAKE_OFF:
                if (stateTimer.seconds() >= intakeOnTime) {
                    // Stop intake
                    intake1.setPower(0);
                    intake2.setPower(0);

                    if (shotsRemaining > 0) {
                        // More shots to go - spin up again
                        stateTimer.reset();
                        flywheelState = FlywheelState.PUSH_UP;
                    } else {
                        // All done - stop flywheel
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }

    // =============== FLYWHEEL CONTROL  ====================
    private void updateFlywheelPower() {
        if (flywheelState != FlywheelState.IDLE) {
            double velocity = Math.abs(fly1.getVelocity());
            double error = targetVelocity - velocity;

            // Calculate power using feedforward + P control
            double power = (targetVelocity * kF) + (error * kP);

            fly1.setPower(power);
            fly2.setPower(power);
        } else {
            fly1.setPower(0);
            fly2.setPower(0);
        }
    }

    // ============== FIRE SHOTS =================
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
            // Reset intake timing for first shot
            intakeOnTime = 0.2;
        }
    }

    // ============== MANUAL SHOOTING POSITIONS =================
    public void setShootingParameters(double velocity, double angle) {
        this.targetVelocity = velocity;
        this.targetAngle = angle;
        launch.setPosition(angle);
    }

    // ============== AUTO ADJUST =================
    public void setAutoAdjust(boolean enabled) {
        this.autoAdjustEnabled = enabled;
    }

    // ============== CHECK IF ROBOT IS ALIGNED =================
    public boolean isAligned() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return Math.abs(getHorizontalOffset() - targetTx) < ALIGNMENT_THRESHOLD;
        }
        return false;
    }

    // ============== GET HORIZONTAL OFFSET (for rotation correction) =================
    public double getHorizontalOffset() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    // ============== GET ROTATION CORRECTION ANGLE =================
    //Returns the angle the robot needs to rotate
    public double getRotationCorrection() {
        return Math.toRadians(-getHorizontalOffset()); // Negative because tx is positive when target is to the right
    }

    // ============== IS SHOOTER BUSY =================
    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    // ============== RUN INTAKE WHILE PICKING UP ROWS =================
    public void setIntakePower(double power) {
        if (flywheelState == FlywheelState.IDLE) {
            intake1.setPower(power);
            intake2.setPower(power);
        }
    }

    // ============== STOP INTAKE =================
    public void stopIntake() {
        if (flywheelState == FlywheelState.IDLE) {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }

    // ================= TELEMETRY =================

    public String getState() {
        return flywheelState.toString();
    }

    public int getShotsRemaining() {
        return shotsRemaining;
    }

    public double getFlywheelRPM() {
        return (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getDistance() {
        return trigDistance;
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }
}