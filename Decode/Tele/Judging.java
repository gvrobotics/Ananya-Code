package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Test.DistanceSensor;

@TeleOp(name = "Judging", group = "A")
public class Judging extends OpMode {

    private Servo launch;
    private Limelight3A limelight;
    private Drivetrain robot;
    public DcMotorEx fly1, fly2;
    private double P = 0.212, F = 12.199;

    DistanceSensor distanceSensor = new DistanceSensor();

    // Limelight constants
    private static final double LIME_MOUNT_ANGLE = 15.0, LIME_HEIGHT = 13.0, TAG_HEIGHT = 38.0;         // inches

    // Lookup tables for distance -> angle
    private final double[] distances = {30, 40, 50, 60, 75, 137, 165, 172};
    private final double[] angles = {1, 0.7, 0.65, 0.6, 0.35, 0.15, 0.15, 0.1};

    // Control constants
    private double kP_rotation = 0.02;    // How fast to turn toward target
    private double targetAngle = 0.5;  // Default angle
    private double trigDistance = -1;   // Distance to target

    @Override
    public void init() {
        // Initialize drivetrain
        robot = new Drivetrain(hardwareMap);

        // Initialize servo
        launch = hardwareMap.get(Servo.class, "l");
        launch.setDirection(Servo.Direction.FORWARD);
        launch.setPosition(0.5);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        distanceSensor.init(hardwareMap);

        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fly1.setVelocityPIDFCoefficients(P, 0, 0, F);
        fly2.setVelocityPIDFCoefficients(P, 0, 0, F);

        fly1.setPower(0);
        fly2.setPower(0);
    }

    @Override
    public void loop() {
        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();

        double rotationCorrection = 0;

        // Check if we have a valid AprilTag detection
        if (result != null && result.isValid()) {
            // Calculate distance using trigonometry
            double ty = result.getTy();  // Vertical angle offset
            double tx = result.getTx();  // Horizontal angle offset (left/right)

            double angleRad = Math.toRadians(LIME_MOUNT_ANGLE + ty);
            trigDistance = (TAG_HEIGHT - LIME_HEIGHT) / Math.tan(angleRad);

            // Look up the appropriate launch angle for this distance
            targetAngle = interpolate(distances, angles, trigDistance);

            // Set the launch angle
            launch.setPosition(targetAngle);

            // ===== ROTATION TO TRACK TAG =====
            // If tx is positive, target is to the right, so rotate right
            // If tx is negative, target is to the left, so rotate left
            rotationCorrection = tx * kP_rotation;
            rotationCorrection = Math.max(-0.5, Math.min(0.5, rotationCorrection)); // Clamp

            // Telemetry
            telemetry.addData("Distance (inches)", "%.1f", trigDistance);
            telemetry.addData("Launch Angle", "%.3f", targetAngle);
            telemetry.addLine();
            telemetry.addData("TX (horizontal offset)", "%.2f°", tx);
            telemetry.addData("Rotation Speed", "%.3f", rotationCorrection);
        } else {
            // No target detected - stop robot
            telemetry.addLine("NO TARGET");
            telemetry.addData("Launch Angle", "%.3f (default)", targetAngle);
            trigDistance = -1;
        }

        // Rotate robot to keep tag centered
        robot.autoMove(0, 0, rotationCorrection);

        if (distanceSensor.getDistance() < 12) {
            gamepad1.rumble(1.0, 1.0, 300);
            fly1.setPower(0.3);
            fly2.setPower(0.3);
            telemetry.addData("Distance: ", distanceSensor.getDistance());
            telemetry.addData("Status: ", "FULL");
        } else {
            fly1.setPower(0);
            fly2.setPower(0);
            telemetry.addData("Distance: ", distanceSensor.getDistance());
            telemetry.addData("Status: ", "NOT FULL");
        }

        telemetry.update();
    }

    private double interpolate(double[] x, double[] y, double value) {
        // Clamp to range
        if (value <= x[0]) return y[0];
        if (value >= x[x.length - 1]) return y[y.length - 1];

        // Find the right segment and interpolate
        for (int i = 0; i < x.length - 1; i++) {
            if (value >= x[i] && value <= x[i + 1]) {
                double ratio = (value - x[i]) / (x[i + 1] - x[i]);
                return y[i] + ratio * (y[i + 1] - y[i]);
            }
        }

        return y[0];
    }
}