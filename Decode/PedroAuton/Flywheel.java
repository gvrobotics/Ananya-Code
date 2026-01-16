package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tele.Tele5440;

public class Flywheel {
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        PUSH_FORWARD,
        PUSH_BACK,
        INTAKE_ON
    }

    private FlywheelState flywheelState;

    //-------- PUSHER CONSTANTS --------
    private double PUSH_UP_ANGLE = 0.1;
    private double PUSH_DOWN_ANGLE = 0.5;
    private double PUSH_UP_TIME = 0.7;
    //private double PUSH_DOWN_TIME = 0.2; // time to go to initial position


    //-------- FLYWHEEL CONSTANTS --------
    private int shotsRemaining = 0;
    private double flywheelVelocity = 1800;
    private double MIN_FLYWHEEL_RPM = 1150, TARGET_FLYWHEEL_RPM = 1500;
    private double FLYWHEEL_MAX_SPINUP_TIME = 3;
    private double P = 0.212, F = 12.199;


    public void init(HardwareMap hwMap) {
        push1 = hwMap.get(Servo.class, "p1");
        fly1 = hwMap.get(DcMotorEx.class, "f1");
        fly2 = hwMap.get(DcMotorEx.class, "f2");
        intake1 = hwMap.get(DcMotorEx.class, "i1");
        intake2 = hwMap.get(DcMotorEx.class, "i2");

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly1.setVelocityPIDFCoefficients(P, 0, 0, F);
        fly2.setVelocityPIDFCoefficients(P, 0, 0, F);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;
        fly1.setPower(0);
        fly2.setPower(0);
        push1.setPosition(0.5);
        intake1.setVelocity(0);
        intake2.setVelocity(0);
    }

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    push1.setPosition(PUSH_DOWN_ANGLE);
                    fly1.setVelocity(TARGET_FLYWHEEL_RPM);
                    fly2.setVelocity(TARGET_FLYWHEEL_RPM);

                    stateTimer.reset();
                    flywheelState = flywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP: {
                fly1.setVelocity(TARGET_FLYWHEEL_RPM);
                fly2.setVelocity(TARGET_FLYWHEEL_RPM);

                double currentRPM =
                        (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;

                // is flywheel at speed or past spin up time
                if (currentRPM > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    push1.setPosition(PUSH_UP_ANGLE);
                    stateTimer.reset();

                    flywheelState = flywheelState.LAUNCH;
                }
            }
            break;

            case LAUNCH: {
                if (stateTimer.seconds() > PUSH_UP_TIME) {
                    shotsRemaining--;
                    push1.setPosition(PUSH_DOWN_ANGLE);
                    timer.reset();
                    stateTimer.reset();
                    flywheelState = flywheelState.PUSH_FORWARD;
                }
            }
            break;

            case PUSH_FORWARD:
                if (timer.seconds() >= 0.7) {
                    // Pusher down
                    push1.setPosition(0.5);
                    timer.reset();
                    flywheelState = FlywheelState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() >= 0.7) {
                    // Turn intake on
                    intake1.setVelocity(1500);
                    intake2.setVelocity(1500);
                    timer.reset();
                    stateTimer.reset();
                    flywheelState = FlywheelState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() >= 0.2) {
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


