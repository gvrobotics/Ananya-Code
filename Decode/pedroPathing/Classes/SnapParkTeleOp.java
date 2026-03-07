package org.firstinspires.ftc.teamcode.pedroPathing.Classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "B")
@Configurable
public class SnapParkTeleOp extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;

    private boolean automatedDrive = false;
    private double autoStartTime = 0;

    // ===== PARK POSES =====
    private static final Pose RED_PARK = new Pose(38.5, 33.5, Math.toRadians(0));
    private static final Pose BLUE_PARK = new Pose(105.5, 33.5, Math.toRadians(0));

    private PathChain redMove, blueMove;


    public void buildPaths() {

        redMove = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, RED_PARK))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                RED_PARK.getHeading(),
                                0.7))
                .build();

        blueMove = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, BLUE_PARK))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                BLUE_PARK.getHeading(),
                                0.7))
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();

        // ===== MANUAL DRIVE =====
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 1.2,
                    gamepad1.left_stick_x * 1.4,
                    gamepad1.right_stick_x * 1.2,
                    false
            );
        }

        // SNAP TO NEAREST PARK (GAMEPAD2 Y)
        if (gamepad2.yWasPressed() && !automatedDrive) {

            double distRed = distance(currentPose, RED_PARK);
            double distBlue = distance(currentPose, BLUE_PARK);

            if (distRed < distBlue) {
                follower.followPath(redMove, true);
            } else {
                follower.followPath(blueMove, true);
            }

            automatedDrive = true;
            autoStartTime = getRuntime();
        }

        // ===== MANUAL BLUE (X) =====
        if (gamepad2.xWasPressed() && !automatedDrive) {
            follower.followPath(blueMove, true);
            automatedDrive = true;
            autoStartTime = getRuntime();
        }

        // ===== MANUAL RED (B) =====
        if (gamepad2.bWasPressed() && !automatedDrive) {
            follower.followPath(redMove, true);
            automatedDrive = true;
            autoStartTime = getRuntime();
        }


        // EXIT CONDITIONS
        if (automatedDrive) {

            boolean timeout = (getRuntime() - autoStartTime) > 2.75;
            boolean finished = !follower.isBusy();
            boolean cancelled = gamepad2.aWasPressed();

            if (timeout || finished || cancelled) {
                follower.breakFollowing();
                follower.startTeleopDrive();
                automatedDrive = false;
            }
        }

        // ================= TELEMETRY =================
        telemetryM.debug("=== SNAP PARK DEBUG ===");
        telemetryM.debug("X", String.format("%.1f", currentPose.getX()));
        telemetryM.debug("Y", String.format("%.1f", currentPose.getY()));
        telemetryM.debug("Heading", String.format("%.1f°",
                Math.toDegrees(currentPose.getHeading())));
        telemetryM.debug("Mode", automatedDrive ? "AUTO-PARKING" : "MANUAL");

        telemetry.addData("Pose",
                "(%.1f, %.1f) @ %.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Mode", automatedDrive ? "AUTO-PARKING" : "MANUAL");
        telemetry.addData("Y", "Snap to nearest park");
        telemetry.addData("X/B", "Manual blue/red");
        telemetry.addData("A", "Cancel");
        telemetry.update();

        telemetryM.update();
    }

    // ================= HELPER =================
    private double distance(Pose a, Pose b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }
}
