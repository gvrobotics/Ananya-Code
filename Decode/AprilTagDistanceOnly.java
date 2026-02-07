package org.firstinspires.ftc.teamcode.AprilTags;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "AprilTag Distance Only")
public class AprilTagDistanceOnly extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D tagPose = result.getBotpose_MT2(); // robot relative to tag

            // Limelight gives meters → convert to inches
            double x_in = tagPose.getPosition().x * 39.3701; // forward
            double y_in = tagPose.getPosition().y * 39.3701; // left/right

            // distance from robot to tag (Pythagorean)
            double distance_in = Math.hypot(x_in, y_in);

            telemetry.addData("Distance to Tag (in)", distance_in);
            telemetry.addData("X (in)", x_in);
            telemetry.addData("Y (in)", y_in);
        } else {
            telemetry.addLine("No valid AprilTag detected");
        }

        telemetry.update();
    }
}
