package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "New Shooter Test", group = "B")
public class ShooterFPTest extends OpMode {

    DcMotorEx fly1, fly2;
    Servo push1, push2, launch;

    Limelight3A limelight;
    IMU imu;

    // ===== TUNING =====
    double targetVelocity = 1500;
    double kF = 1 / 2320.0;
    double kP = 0.0005;

    boolean shooterOn = false;
    boolean prevRB = false;
    boolean pushOn = false;
    boolean prevA = false, currA = false;

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientation));
        limelight.pipelineSwitch(0);
        limelight.start();

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        push1.setPosition(0.4);
        push2.setPosition(0);
        launch.setPosition(0.5);
    }

    @Override
    public void loop() {

        // ===== Update Limelight Orientation =====
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        // ===== PUSH TOGGLE =====
        prevA = currA;
        currA = gamepad2.a;

        if (currA && !prevA) {
            pushOn = !pushOn;

            if (pushOn) {
                push1.setPosition(0.7);
                push2.setPosition(0.25);
            } else {
                push1.setPosition(0.4);
                push2.setPosition(0);
            }
        }

        // ===== LAUNCH Adjust =====
        if (gamepad1.dpad_up) launch.setPosition(launch.getPosition() + 0.01);
        if (gamepad1.dpad_down) launch.setPosition(launch.getPosition() - 0.01);

        // ===== SHOOTER TOGGLE =====
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) shooterOn = !shooterOn;
        prevRB = rb;

        double velocity = Math.abs(fly1.getVelocity());
        double vel = 1000;
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            power = (targetVelocity * kF) + (error * kP);
        }

        fly1.setPower(vel);
        fly2.setPower(vel);

        if (gamepad1.dpad_left) vel += 10;
        if (gamepad1.dpad_right) vel -= 10;

        // ===== Distance from AprilTag =====
        double distanceInches = getDistanceFromLimelight();

        // ===== TELEMETRY =====
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        //telemetry.addData("Target Vel", targetVelocity);
        //telemetry.addData("Power", power);
        telemetry.addData("Distance (in)", distanceInches);
        telemetry.addData("Velocity", vel);
        telemetry.addData("Launch", launch.getPosition());

        telemetry.addLine("==== PUSH ====");
        telemetry.addData("Pusher", pushOn ? "UP" : "DOWN");

        telemetry.update();
    }

    // ===== Distance Helper =====

    private double getDistanceFromLimelight() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D pose = result.getBotpose_MT2();

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;

            double meters = Math.hypot(x, y);
            return meters * 39.3701;
        }

        return -1; // no tag detected
    }
}
