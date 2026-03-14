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
    private final double PUSHED_UP_TIME = 0.15, PUSHED_DOWN_TIME = 0.05;
    private double intakeOnTime = 0.15;

    // ================= FLYWHEEL CONTROL =================
    private int shotsRemaining = 0;
    private final double kF = 0.00052, kP = 0.0026;
    private double targetVelocity, targetAngle;
    private boolean flywheelEnabled = true;

    // ================= LIMELIGHT =================
    private static final double LIME_MOUNT_ANGLE = 15.0, LIME_HEIGHT = 13.0, TAG_HEIGHT = 38.0;
    private double trigDistance = -1, lastKnownDistance = -1;
    private boolean autoAdjustEnabled = false;

    // ================= LOOKUP TABLES =================
    private final double[] distances = {30, 40, 50, 60, 75, 137, 165, 172};
    private final double[] angles    = {1, 0.7, 0.65, 0.6, 0.35, 0.15, 0.15, 0.1};
    private final double[] velocities = {820, 860, 880, 900, 1020, 1160, 1220, 1230};

    public void init(HardwareMap hwMap) {
        fly1 = hwMap.get(DcMotorEx.class, "f1");
        fly2 = hwMap.get(DcMotorEx.class, "f2");
        intake1 = hwMap.get(DcMotorEx.class, "i1");
        intake2 = hwMap.get(DcMotorEx.class, "i2");

        push1 = hwMap.get(Servo.class, "p1");
        push2 = hwMap.get(Servo.class, "p2");
        launch = hwMap.get(Servo.class, "l");

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setPower(0);
        intake2.setPower(0);
        push1.setPosition(PUSH_DOWN_1);
        push2.setPosition(PUSH_DOWN_2);
        launch.setPosition(0.7);

        // Flywheel always on - needs a valid targetVelocity set before useful
        // Call setShootingParameters() or setAutoAdjust(true) to provide targets
    }

    // ================= LINEAR INTERPOLATION =================
    private double interpolate(double[] x, double[] y, double value) {
        if (value <= x[0]) return y[0];
        if (value >= x[x.length - 1]) return y[y.length - 1];
        for (int i = 0; i < x.length - 1; i++) {
            if (value >= x[i] && value <= x[i + 1]) {
                double ratio = (value - x[i]) / (x[i + 1] - x[i]);
                return y[i] + ratio * (y[i + 1] - y[i]);
            }
        }
        return y[0];
    }

    // ================= LIMELIGHT UPDATE =================
    public void updateLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Distance calculations
            double ty = result.getTy();
            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);
            trigDistance = (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);

            lastKnownDistance = trigDistance; // fallback

            if (autoAdjustEnabled) {
                targetAngle = interpolate(distances, angles, trigDistance);
                targetVelocity = interpolate(distances, velocities, trigDistance);
            }

        } else {
            trigDistance = -1;
            // Fall back to last known for autoAdjust
            if (autoAdjustEnabled && lastKnownDistance > 0) {
                targetAngle = interpolate(distances, angles, lastKnownDistance);
                targetVelocity = interpolate(distances, velocities, lastKnownDistance);
            } else {
                targetAngle = 0.684;
                targetVelocity = 866.40;
            }
        }
    }

    // ================= FLYWHEEL POWER (always running) =================
    private void updateFlywheelPower() {
        if (!flywheelEnabled || targetVelocity <= 0) return; // don't spin if no target set

        double velocity = Math.abs(fly1.getVelocity());
        double error = targetVelocity - velocity;
        double power = (targetVelocity * kF) + (error * kP);

        fly1.setPower(power);
        fly2.setPower(power);
    }

    // ================= STATE MACHINE =================
    public void update() {
        updateLimelight();
        updateFlywheelPower();

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    push1.setPosition(PUSH_DOWN_1);
                    push2.setPosition(PUSH_DOWN_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_UP;
                }
                break;

            case PUSH_UP:
                if (autoAdjustEnabled) {
                    launch.setPosition(targetAngle);
                }

                if (getFlywheelRPM() >= targetVelocity * 0.95) {
                    push1.setPosition(PUSH_UP_1);
                    push2.setPosition(PUSH_UP_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_DOWN;
                }
                break;

            case PUSH_DOWN:
                if (stateTimer.seconds() >= PUSHED_UP_TIME) {
                    shotsRemaining--;
                    push1.setPosition(PUSH_DOWN_1);
                    push2.setPosition(PUSH_DOWN_2);
                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (stateTimer.seconds() >= PUSHED_DOWN_TIME) {
                    intake1.setPower(0.8);
                    intake2.setPower(0.8);
                    stateTimer.reset();
                    flywheelState = FlywheelState.INTAKE_OFF;
                }
                break;

            case INTAKE_OFF:
                if (stateTimer.seconds() >= intakeOnTime) {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    stateTimer.reset();
                    flywheelState = shotsRemaining > 0 ? FlywheelState.PUSH_UP : FlywheelState.IDLE;

                    if (shotsRemaining == 2) {
                        intakeOnTime = 0.3;
                    } else {
                        intakeOnTime = 0.2;
                    }
                }
                break;
        }
    }

    // ================= FUNCTIONS =================
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public void setShootingParameters(double velocity, double angle) {
        this.targetVelocity = velocity;
        this.targetAngle = angle;
        launch.setPosition(angle);
    }

    public void setAutoAdjust(boolean enabled) {
        this.autoAdjustEnabled = enabled;
    }

    public void setIntakePower(double power) {
        if (flywheelState == FlywheelState.IDLE) {
            intake1.setPower(power);
            intake2.setPower(power);
        }
    }

    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public void stopFlywheel() {
        flywheelEnabled = false;
        fly1.setPower(0);
        fly2.setPower(0);
    }

    public double getHorizontalOffset() {
        LLResult result = limelight.getLatestResult();
            return result.getTx(); // live value
    }

    // ================= TELEMETRY =================

    public boolean isBusy()            { return flywheelState != FlywheelState.IDLE; }
    public boolean hasValidTarget()    { LLResult r = limelight.getLatestResult(); return r != null && r.isValid(); }
    public String  getState()          { return flywheelState.toString(); }
    public int     getShotsRemaining() { return shotsRemaining; }
    public double  getFlywheelRPM()    { return (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0; }
    public double  getTargetVelocity() { return targetVelocity; }
    public double  getTargetAngle()    { return targetAngle; }
    public double  getDistance()       { return trigDistance > 0 ? trigDistance : lastKnownDistance; }
}