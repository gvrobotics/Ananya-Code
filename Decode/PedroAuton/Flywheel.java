package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        PUSH_DOWN,
        PUSH_BACK,
        INTAKE_ON
    }
    private FlywheelState flywheelState = FlywheelState.IDLE;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    //================= PUSHER CONSTANTS =================
    private double pushUp1 = 0.7, pushDown1 = 0.4, pushUp2 = 0.25, pushDown2 = 0; //push 1 (R) - down is 0.4, up is 0.7 == push 2 (F) - down is 0, up is 0.25
    private double PUSH_UP_TIME = 0.2, PUSH_DOWN_TIME = 0.2;
    private double INTAKE_ON_TIME = 0.2;

    //================= FLYWHEEL CONSTANTS =================
    private int shotsRemaining = 0;
    private double MIN_FLYWHEEL_RPM = 950, TARGET_FLYWHEEL_RPM = 1300;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2.5;
    private double P = 0.212, F = 12.199;
    private double kF = 0.00052;
    private double kP = 0.0026;
    private boolean shooterAtTargetVel = false;

    // ================= LOOKUP TABLE =================
    private final double [] distances = {30, 40, 50, 60, 75, 137, 165, 172};
    private final double [] angles = {1, 0.7, 0.65, 0.6, 0.35, 0.15, 0.15, 0.1};
    private final double [] velocities = {820, 860, 880, 900, 1020, 1160, 1220, 1230};

    private double targetAngle = 0.5;
    private double targetVelocity = 1000;

    // ================= LIMELIGHT VARIABLES =================
    private Limelight3A limelight;
    private static final double LIME_MOUNT_ANGLE = 15.0;
    private static final double LIME_HEIGHT = 13.0;
    private static final double TAG_HEIGHT = 38.0;
    private double trigDistance = -1;


    public void init(HardwareMap hwMap) {
        push1 = hwMap.get(Servo.class, "p1");
        push2 = hwMap.get(Servo.class, "p2");
        fly1 = hwMap.get(DcMotorEx.class, "f1");
        fly2 = hwMap.get(DcMotorEx.class, "f2");
        intake1 = hwMap.get(DcMotorEx.class, "i1");
        intake2 = hwMap.get(DcMotorEx.class, "i2");
        launch = hwMap.get(Servo.class, "l");

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

        // ===== Limelight =====
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize all motors to zero power
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        push1.setPosition(pushDown1);
        push2.setPosition(pushDown2);
        launch.setPosition(0.5);
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

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    fly1.setVelocity(targetVelocity);
                    fly2.setVelocity(targetVelocity);

                    stateTimer.reset();
                    flywheelState = flywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP: {
                fly1.setVelocity(targetVelocity);
                fly2.setVelocity(targetVelocity);

                double currentRPM =
                        (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;

                // is flywheel at speed or past spin up time
                if (currentRPM >= targetVelocity) {
                    push1.setPosition(pushUp1);
                    push2.setPosition(pushUp2);
                    stateTimer.reset();

                    flywheelState = flywheelState.LAUNCH;
                }
            }
            break;

            case LAUNCH: {
                if (stateTimer.seconds() > PUSH_UP_TIME) {
                    shotsRemaining--;
                    push1.setPosition(pushUp1);
                    push2.setPosition(pushUp2);
                    timer.reset();
                    stateTimer.reset();
                    flywheelState = flywheelState.PUSH_DOWN;
                }
            }
            break;

            case PUSH_DOWN:
                if (timer.seconds() >= PUSH_DOWN_TIME) {
                    // Pusher down
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    timer.reset();
                    flywheelState = FlywheelState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() >= 0.2) {
                    // Turn intake on
                    intake1.setPower(0.8);
                    intake2.setPower(0.8);
                    timer.reset();
                    stateTimer.reset();
                    flywheelState = FlywheelState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() >= INTAKE_ON_TIME) {
                    intake1.setVelocity(0);
                    intake2.setVelocity(0);

                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        fly1.setVelocity(0);
                        fly2.setVelocity(0);
                        flywheelState = FlywheelState.IDLE;
                    }

                    if (shotsRemaining == 2) {
                        INTAKE_ON_TIME = 0.4;
                    } else {
                        INTAKE_ON_TIME = 0.2;
                    }
                }
                break;
        }
    }

    public void fireShots (int numberofshots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberofshots;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    public String getState() {
        return flywheelState.toString();
    }

    public int getShotsRemaining() {
        return shotsRemaining;
    }

    public double getFlywheelRPM() {
        return (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;
    }
}
