package org.firstinspires.ftc.teamcode.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "AprilTag Distance Limelight")
public class AprilTagDistanceLimelight extends OpMode {

    private Limelight3A limelight;

    // Limelight mounting constants
    private static final double LIME_MOUNT_ANGLE_DEG = 15.0; // Limelight angle from vertical
    private static final double LIME_HEIGHT_IN = 13.0;       // Limelight height from floor
    private static final double TAG_HEIGHT_IN = 38.0;        // AprilTag height from floor

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight Initialized");
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
            // Get vertical offset (ty) from NetworkTables
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry tyEntry = table.getEntry("ty");
            double ty = tyEntry.getDouble(0.0);

            // Calculate distance using vertical angle
            double angleToTargetDeg = LIME_MOUNT_ANGLE_DEG + ty;
            double angleToTargetRad = Math.toRadians(angleToTargetDeg);

            double distanceInches = (TAG_HEIGHT_IN - LIME_HEIGHT_IN) / Math.tan(angleToTargetRad);

            telemetry.addLine("AprilTag detected!");
            telemetry.addData("Vertical Offset (ty)", ty);
            telemetry.addData("Distance to Tag (inches)", distanceInches);

        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }
}
