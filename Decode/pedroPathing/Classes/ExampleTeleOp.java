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

@TeleOp (group = "B")
@Configurable
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    private boolean automatedDrive = false;
    private TelemetryManager telemetryM;

    private PathChain redmove, bluemove;

    public void buildPaths() {
        redmove = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(38.5, 33.5)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        bluemove = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(105.5, 33.5)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
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

        // ===== Manual Drive =====
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 1.2,
                    gamepad1.left_stick_x * 1.4,
                    gamepad1.right_stick_x * 1.2,
                    false // Field Centric
            );
        }

        // ===== Go to BLUE Park Zone (Press X) =====
        if (gamepad2.xWasPressed() && !automatedDrive) {
            follower.followPath(bluemove, true);
            automatedDrive = true;
        }

        // ===== Go to RED Park Zone (Press B) =====
        if (gamepad2.bWasPressed() && !automatedDrive) {
            follower.followPath(redmove, true);
            automatedDrive = true;
        }

        // ===== Stop Auto Path (Press A OR path finished) =====
        if (automatedDrive && (gamepad2.aWasPressed() || !follower.isBusy())) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Debug info
        telemetryM.debug("=== ROBOT STATUS ===");
        telemetryM.debug("X", String.format("%.1f", follower.getPose().getX()));
        telemetryM.debug("Y", String.format("%.1f", follower.getPose().getY()));
        telemetryM.debug("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("Mode", automatedDrive ? "AUTO-PARKING" : "MANUAL");
        telemetryM.debug("");
        telemetryM.debug("=== CONTROLS ===");
        telemetryM.debug("Gamepad 2 X", "Blue Park Zone (105.5, 33.5)");
        telemetryM.debug("Gamepad 2 B", "Red Park Zone (38.5, 33.5)");
        telemetryM.debug("Gamepad 2 A", "Cancel & Manual Control");

        telemetry.addData("Position", String.format("(%.1f, %.1f) @ %.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Mode", automatedDrive ? "AUTO-PARKING" : "MANUAL");
        telemetry.addData("", "");
        telemetry.addData("X", "Blue Park (105.5, 33.5)");
        telemetry.addData("B", "Red Park (38.5, 33.5)");
        telemetry.addData("A", "Cancel");
        telemetry.update();

        telemetryM.update();
    }
}