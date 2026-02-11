package org.firstinspires.ftc.teamcode.AprilTags;
import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor; // Logic for understanding April Tags
    private VisionPortal visionPortal; // Allows for us to open up vision portals to use visual processors from webcam
    private List<AprilTagDetection> detectedTags = new ArrayList<>(); // List to store tags and information about them (pos, distance, angle, elevation, etc.)
    private Telemetry telemetry; // Displays information
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        // Define information for this builder
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true) // show ID on screen
                .setDrawTagOutline(true)
                .setDrawAxes(true) // draw x, y and z axes
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        // create vision portal to accept April Tags
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1")); // gets camera from configuration
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build(); // creates vision portal
    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
            // ranger = center of camera to center of tag
            // bearing = angle of deflection away form that object
            // elevation = how far up you are based on that
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        // loops through detected tags until we find tag with specific ID
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop (){
        if (visionPortal !=null) { // check if theres an instance of vision portal still running
            visionPortal.close();
        }
    }
}
