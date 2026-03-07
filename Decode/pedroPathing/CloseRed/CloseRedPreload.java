package org.firstinspires.ftc.teamcode.pedroPathing.CloseRed;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Classes.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Classes.Flywheel;

@Autonomous(group = "Close Red", name = "4. Close Red Preload", preselectTeleOp = "Tele 5440")
public class CloseRedPreload extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Flywheel shooter = new Flywheel();

    // Timers
    private Timer pathTimer, opModeTimer;

    // States
    private boolean pathStarted = false, shotsTriggered = false, alignmentChecked = false;

    // Shooting parameters (just in case auto aim doesn't work)
    private static final double SHOOTING_VELOCITY = 880, SHOOTING_ANGLE = 0.67;
    private boolean USE_AUTO_ADJUST = true; // Set to false for manual parameters

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        ALIGN_AND_SHOOT_PRELOAD,
        SHOOT_TO_ENDPOS,
        COMPLETE
    }
    PathState pathState;

    // Poses
    private final Pose startPose = new Pose(117.5, 129.2, Math.toRadians(45));
    private final Pose shootPose = new Pose(92.6, 99.3, Math.toRadians(40));
    private final Pose endPose = new Pose(95, 121.3, Math.toRadians(0));

    // Paths
    private PathChain driveStartPosShootPos, driveShootPosEndPose;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.ALIGN_AND_SHOOT_PRELOAD);
                break;

            case ALIGN_AND_SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    handleShoot(PathState.SHOOT_TO_ENDPOS);
                }
                break;

            case SHOOT_TO_ENDPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosEndPose, true);
                    setPathState(PathState.COMPLETE);
                }
                break;

            case COMPLETE:
                telemetry.addLine("YAY DONE");
                break;

            default:
                telemetry.addLine("ERROR: Invalid State");
                break;
        }
    }

    // ============== Shoot ==============
    private void handleShoot(PathState nextState) {
        if (!alignmentChecked) {
            setupShot();
            alignmentChecked = true;
        }

        if (!shotsTriggered) {
            shooter.fireShots(3);
            shotsTriggered = true;
        } else if (!shooter.isBusy()) {
            setPathState(nextState);
        }
    }

    private void setupShot() {
        shooter.setAutoAdjust(USE_AUTO_ADJUST);

        // Always set fallback parameters in case Limelight fails
        if (!shooter.hasValidTarget()) {
            shooter.setShootingParameters(SHOOTING_VELOCITY, SHOOTING_ANGLE);
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
        shotsTriggered = false;
        alignmentChecked = false;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Auto-Adjust", USE_AUTO_ADJUST ? "ENABLED" : "DISABLED");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();

        // Telemetry
        panelsTelemetry.debug("=== PATH ===");
        panelsTelemetry.debug("State", pathState.toString());
        panelsTelemetry.debug("Time", pathTimer.getElapsedTimeSeconds());

        panelsTelemetry.debug("=== POSITION ===");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        panelsTelemetry.debug("=== SHOOTER ===");
        panelsTelemetry.debug("State", shooter.getState());
        panelsTelemetry.debug("Shots Left", shooter.getShotsRemaining());
        panelsTelemetry.debug("Flywheel RPM", String.format("%.0f", shooter.getFlywheelRPM()));
        panelsTelemetry.debug("Target RPM", String.format("%.0f", shooter.getTargetVelocity()));

        panelsTelemetry.debug("=== LIMELIGHT ===");
        panelsTelemetry.debug("Valid Target", shooter.hasValidTarget() ? "YES" : "NO");
        panelsTelemetry.debug("Distance (in)", String.format("%.1f", shooter.getDistance()));
        panelsTelemetry.debug("Horizontal Offset", String.format("%.2f°", shooter.getHorizontalOffset()));
        panelsTelemetry.debug("Launch Angle", String.format("%.2f", shooter.getTargetAngle()));
        panelsTelemetry.update(telemetry);
    }
}