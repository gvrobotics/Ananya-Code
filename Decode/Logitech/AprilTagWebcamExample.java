package org.firstinspires.ftc.teamcode.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp (name = "AprilTagWebcamExample", group = "AprilTag")
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam webcam = new AprilTagWebcam();

    @Override
    public void init() {
        webcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // update the vision portal
        webcam.update();
        AprilTagDetection id24 = webcam.getTagBySpecificID(24);
        webcam.displayDetectionTelemetry(id24);
        telemetry.update();
    }
}
