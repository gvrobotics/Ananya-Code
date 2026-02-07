package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter + Limelight")
public class ShooterLimelight extends OpMode {

    // ===== Hardware =====
    DcMotorEx fly1, fly2, intake1, intake2;
    Servo push1, push2, launch;
    Limelight3A limelight;

    // ===== Intake toggle =====
    boolean intakeOn = false;
    boolean prevLB = false;

    // ===== Shooter toggle =====
    boolean shooterOn = false;
    boolean prevRB = false;

    // ===== B button launch =====
    boolean prevB = false;

    // ===== D-pad edge detect =====
    boolean prevUp = false;
    boolean prevDown = false;

    // ===== Shooter velocity =====
    double shooterVelocity = 1500;

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

        push1.setPosition(pushDown1);
        push2.setPosition(pushDown2);
        launch.setPosition(0.5);
    }

    @Override
    public void loop() {

        // ================= INTAKE TOGGLE =================
        boolean currLB = gamepad1.left_bumper;
        if (currLB && !prevLB) intakeOn = !intakeOn;
        prevLB = currLB;

        double intakePower = intakeOn ? 0.4 : 0;
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        // ================= SHOOTER TOGGLE =================
        boolean currRB = gamepad1.right_bumper;
        if (currRB && !prevRB) shooterOn = !shooterOn;
        prevRB = currRB;

        if (shooterOn) {
            fly1.setVelocity(shooterVelocity);
            fly2.setVelocity(shooterVelocity);
        } else {
            fly1.setVelocity(0);
            fly2.setVelocity(0);
        }

        // ================= VELOCITY ADJUST =================
        boolean currUp = gamepad1.dpad_up;
        boolean currDown = gamepad1.dpad_down;

        if (currUp && !prevUp) shooterVelocity += 10;
        if (currDown && !prevDown) shooterVelocity -= 10;

        prevUp = currUp;
        prevDown = currDown;

        // ================= LAUNCH ANGLE ADJUST =================
        if (gamepad1.dpadUpWasPressed())
            launch.setPosition(launch.getPosition() + 0.01);

        if (gamepad1.dpadDownWasPressed())
            launch.setPosition(launch.getPosition() - 0.01);

        // ================= B BUTTON SHOOT =================
        boolean currB = gamepad1.b;

        if (currB && !prevB && launchState == LaunchState.IDLE) {

            intakeOn = false;
            shotsRemaining = 1;

            push1.setPosition(pushUp1);
            push2.setPosition(pushUp2);

            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }

        prevB = currB;

        // ===== Launch State Machine =====
        switch (launchState) {

            case PUSH_DOWN:
                if (timer.seconds() > 0.2) {
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    timer.reset();
                    launchState = LaunchState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() > 0.2) {
                    intake1.setVelocity(1500);
                    intake2.setVelocity(1500);
                    timer.reset();
                    launchState = LaunchState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() > 0.5) {
                    intake1.setVelocity(0);
                    intake2.setVelocity(0);
                    launchState = LaunchState.IDLE;
                }
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
        telemetry.addData("Shooter Velocity", shooterVelocity);
        telemetry.addData("Launch Angle", launch.getPosition());
        telemetry.addData("Intake Power", intakePower);
        telemetry.update();
    }
}
