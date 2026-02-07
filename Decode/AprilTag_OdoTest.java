package org.firstinspires.ftc.teamcode.AprilTags;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;

@TeleOp(name = "AprilTag + Odo Test")
public class AprilTag_OdoTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private GoBildaPinpointDriver odo;

    // smoothing factor for AprilTag correction
    private static final double TAG_BLEND = 0.15;

    @Override
    public void init() {

        // ===== Hardware =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // ===== Limelight =====
        limelight.pipelineSwitch(0);

        // ===== IMU (used internally by odo) =====
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(hubOrientation));

        // ===== Odometry setup =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        telemetry.addLine("AprilTag + Odometry Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // ===== Update odometry every loop =====
        odo.update();
        Pose2D odoPose = odo.getPosition();

        double odoHeading = odoPose.getHeading(AngleUnit.DEGREES);

        // Feed heading to Limelight
        limelight.updateRobotOrientation(odoHeading);

        // ===== AprilTag detection =====
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D tagPose = result.getBotpose_MT2();

            double tagX = tagPose.getPosition().x;
            double tagY = tagPose.getPosition().y;

            // current odo pose
            double odoX = odoPose.getX(DistanceUnit.INCH);
            double odoY = odoPose.getY(DistanceUnit.INCH);

//
//           double correctedX = odoX * (1 - TAG_BLEND) + tagX * TAG_BLEND;
//           double correctedY = odoY * (1 - TAG_BLEND) + tagY * TAG_BLEND;

            Pose2D correctedPose = new Pose2D(DistanceUnit.INCH, odoX, odoY, AngleUnit.DEGREES, odoHeading);

            odo.setPosition(correctedPose);

            telemetry.addLine("AprilTag correction active");
            // distance info
            telemetry.addData("Tag Distance (m)", Math.hypot(tagX, tagY));
        }
        else {
            telemetry.addLine("No AprilTag visible");
        }

        // ===== Manual reset =====
        if (gamepad1.start) {
            odo.resetPosAndIMU();
        }

        // ===== Telemetry =====
        telemetry.addLine("---- ODOMETRY ----");
        telemetry.addData("X (in)", odoPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", odoPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", odoHeading);
        telemetry.update();
    }
}
